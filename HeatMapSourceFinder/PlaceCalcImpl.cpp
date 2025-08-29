#include "PlaceCalcImpl.h"
#include "SourceEstimator.h"

#include <cmath>
#include <limits>
#include <ctime>
#include <mutex>
#include <algorithm> // for min/max
#include <iostream>
#include <regex>
#include <fstream>
#include <deque>
#include <numeric>

const double M_PI = 3.14159265358979323846;

PlaceCalcImpl::PlaceCalcImpl() : sigmaSmoothed(1.0) {
}

PlaceCalcImpl::~PlaceCalcImpl() {}

bool PlaceCalcImpl::loadMeasuresFromFile(const std::string& filename, const std::string& sensorId)
{
    m_lastSensorId = sensorId;

    std::ifstream file(filename);
    if (!file.is_open()) {
        notifyError("Ошибка: не удалось открыть файл для загрузки мер");
        return false;
    }

    // Регэкспы — покрывают строки с ID и без ID.
    // Примеры:
    // "12:34:56.789 sensorId add measure 55.75 37.61 123 (t 1620000000)"
    // "1 add measure 55.75 37.61 123 (t 1620000000)"
    std::regex re_with_id(R"(^\s*\d{2}:\d{2}:\d{2}\.\d{3}\s+(\S+)\.?\s+add\s+measure\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+(\d+).*\(t\s+(\d+)\))");
    std::regex re_simple(R"(^\s*\d+\s+add\s+measure\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+(\d+).*\(t\s+(\d+)\))");

    std::string line;
    bool any = false;
    while (std::getline(file, line)) {
        std::smatch match;
        if (std::regex_search(line, match, re_with_id)) {
            double lat = std::stod(match[2].str());
            double lon = std::stod(match[3].str());
            int dist = static_cast<int>(std::lround(std::stod(match[4].str())));
            time_t ts = static_cast<time_t>(std::stoll(match[5].str()));
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (sensorId.empty() || match[1].str() == sensorId) {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_measures.push_back({ lat, lon, dist, ts });
                }

            }
            any = true;
        }
        else if (std::regex_search(line, match, re_simple)) {
            double lat = std::stod(match[1].str());
            double lon = std::stod(match[2].str());
            int dist = static_cast<int>(std::lround(std::stod(match[3].str())));
            time_t ts = static_cast<time_t>(std::stoll(match[4].str()));
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (sensorId.empty() || match[1].str() == sensorId) {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_measures.push_back({ lat, lon, dist, ts });
                }

            }
            any = true;
        }
        else {
            // Для отладки — можно временно раскомментировать:
            // std::cerr << "[PlaceCalcImpl::loadMeasuresFromFile] skip: " << line << "\n";
        }
    }

    if (!any) {
        notifyError("Не найдено ни одного измерения в файле");
        return false;
    }
    return true;
}

void PlaceCalcImpl::setListener(PlaceCalcListener* listener) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_listener = listener;
}

void PlaceCalcImpl::setFindStep(unsigned int step_m) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_step = step_m > 0 ? step_m : 10;
}

void PlaceCalcImpl::setMeasureLifeTime(int seconds) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_measureLifeTime = seconds > 0 ? seconds : 60;
}

void PlaceCalcImpl::reset() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_measures.clear();
}

// Измененный метод addMeasure
void PlaceCalcImpl::addMeasure(double ownLatitude, double ownLongitude, int distance_m, const time_t& dateTime) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_measures.push_back({ ownLatitude, ownLongitude, distance_m, dateTime });
    updateSigmaWindow(distance_m); // Обновляем окно и sigma
}

void PlaceCalcImpl::notifyError(const char* message) {
    if (m_listener) {
        m_listener->onError(message);
    }
}

void PlaceCalcImpl::notifyResult(double latitude,
    double longitude,
    const std::vector<float>& probabilityZone,
    double zoneLeftTopLatitude,
    double zoneLeftTopLongitude,
    double zoneRightBottomLatitude,
    double zoneRightBottomLongitude,
    unsigned int zoneWidth,
    unsigned int zoneHeight) {
    if (m_listener) {
        m_listener->onCalcResult(latitude,
            longitude,
            const_cast<float*>(probabilityZone.data()),
            zoneLeftTopLatitude,
            zoneLeftTopLongitude,
            zoneRightBottomLatitude,
            zoneRightBottomLongitude,
            zoneWidth,
            zoneHeight);
    }
}

void PlaceCalcImpl::setGridSize(unsigned int gridWidth, unsigned int gridHeight) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (gridWidth == 0) gridWidth = 1;
    if (gridHeight == 0) gridHeight = 1;
    m_gridWidth = gridWidth;
    m_gridHeight = gridHeight;
}


void PlaceCalcImpl::addMeasurement(double value) {
    if (window.size() >= windowSize) {
        window.pop_front();
    }
    window.push_back(value);

    // шаг 1: чистим окно
    denoiseWindow(window);

    // шаг 2: считаем robust σ
    double newSigma = computeRobustSigma(window);

    // шаг 3: сглаживаем σ через EMA
    if (sigmaSmoothed <= 0.0)
        sigmaSmoothed = newSigma;
    else
        sigmaSmoothed = alpha * newSigma + (1.0 - alpha) * sigmaSmoothed;
}

void PlaceCalcImpl::denoiseWindow(std::deque<double>& w) {
    if (w.size() < 5) return;

    // считаем медиану
    std::vector<double> sorted(w.begin(), w.end());
    std::sort(sorted.begin(), sorted.end());
    double med = sorted[sorted.size() / 2];

    // считаем MAD (median absolute deviation)
    std::vector<double> devs;
    devs.reserve(sorted.size());
    for (double v : sorted) devs.push_back(std::abs(v - med));
    std::sort(devs.begin(), devs.end());
    double mad = devs[devs.size() / 2];
    double robustSigma = 1.4826 * mad;

    // выкидываем из окна всё, что дальше 3*robustSigma от медианы
    w.erase(std::remove_if(w.begin(), w.end(),
        [&](double v) { return std::abs(v - med) > 3.0 * robustSigma; }),
        w.end());
}

double PlaceCalcImpl::computeRobustSigma(const std::deque<double>& w) {
    if (w.empty()) return 1.0;

    std::vector<double> sorted(w.begin(), w.end());
    std::sort(sorted.begin(), sorted.end());

    // обрежем 10% хвостов
    size_t n = sorted.size();
    size_t cut = static_cast<size_t>(n * trimPercent);
    if (2 * cut >= n) return 1.0; // слишком мало данных

    std::vector<double> trimmed(sorted.begin() + cut, sorted.end() - cut);

    double mean = std::accumulate(trimmed.begin(), trimmed.end(), 0.0) / trimmed.size();
    double sqsum = 0.0;
    for (double v : trimmed) sqsum += (v - mean) * (v - mean);
    return std::sqrt(sqsum / trimmed.size());
}

// Новый метод updateSigmaWindow
void PlaceCalcImpl::updateSigmaWindow(int distance_m) {
    double distance_val = static_cast<double>(distance_m);
    if (window.size() >= windowSize) {
        window.pop_front();
    }
    window.push_back(distance_val);
    denoiseWindow(window);
    double newSigma = computeRobustSigma(window);
    if (sigmaSmoothed <= 0.0) {
        sigmaSmoothed = newSigma;
    }
    else {
        sigmaSmoothed = alpha * newSigma + (1.0 - alpha) * sigmaSmoothed;
    }
}

// -----------------------------------------------------------------------------
// PlaceCalcImpl::calc() — стартует рендер только когда накоплено > 30 мер
// -----------------------------------------------------------------------------
void PlaceCalcImpl::calc() {
    auto start = std::chrono::steady_clock::now();

    // --- Копируем меры ---
    std::vector<Measure> measures_copy;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        measures_copy = m_measures;
        m_lastUsedMeasures = measures_copy.size();
    }

    const size_t WINDOW = 30;
    if (measures_copy.size() <= WINDOW) {
        m_lastStddevMeters = 0.0;
        return;
    }

    // --- Используем предварительно вычисленное значение sigma ---
    double sigmaMeters = sigmaSmoothed;

    // --- Построение матрицы вероятностей ---
    auto buildMatrix = [&](double lat_min_in, double lat_max_in,
        double lon_min_in, double lon_max_in,
        unsigned int& outGridW, unsigned int& outGridH,
        double& out_cell_lat_deg, double& out_cell_lon_deg)
        -> std::vector<std::vector<double>>
        {
            constexpr double meters_per_degree_lat = 111320.0;
            double avg_lat_rad = ((lat_min_in + lat_max_in) / 2.0) * M_PI / 180.0;
            double meters_per_degree_lon = meters_per_degree_lat * std::cos(avg_lat_rad);

            double lat_range_m = std::max((lat_max_in - lat_min_in) * meters_per_degree_lat, 1e-6);
            double lon_range_m = std::max((lon_max_in - lon_min_in) * meters_per_degree_lon, 1e-6);

            double targetCellSizeMeters = sigmaMeters > 0.0
                ? std::clamp(sigmaMeters / 3.0, 5.0, 20.0)
                : 20.0;

            const double MAX_CELLS = 250000.0;
            unsigned int gridHtmp = std::max(1u, (unsigned int)(lat_range_m / targetCellSizeMeters));
            unsigned int gridWtmp = std::max(1u, (unsigned int)(lon_range_m / targetCellSizeMeters));

            double totalCells = (double)gridWtmp * (double)gridHtmp;
            if (totalCells > MAX_CELLS) {
                double scale = std::sqrt(totalCells / MAX_CELLS);
                targetCellSizeMeters *= scale;
                gridHtmp = std::max(1u, (unsigned int)(lat_range_m / targetCellSizeMeters));
                gridWtmp = std::max(1u, (unsigned int)(lon_range_m / targetCellSizeMeters));
            }

            double meters_per_cell = std::max(lat_range_m / gridHtmp, lon_range_m / gridWtmp);

            unsigned int gridW = std::max(1u, (unsigned int)(lon_range_m / meters_per_cell));
            unsigned int gridH = std::max(1u, (unsigned int)(lat_range_m / meters_per_cell));

            gridW = ((gridW + 9) / 10) * 10;
            gridH = ((gridH + 9) / 10) * 10;

            double cell_lat_deg = (lat_max_in - lat_min_in) / (double)gridH;
            double cell_lon_deg = (lon_max_in - lon_min_in) / (double)gridW;

            // --- Окно последних WINDOW мер ---
            std::vector<Measure> window(measures_copy.end() - WINDOW, measures_copy.end());

            std::vector<Sensor> sensors;
            sensors.reserve(window.size());
            for (const auto& m : window) {
                Sensor s;
                double x_cells = (m.longitude - lon_min_in) / cell_lon_deg;
                double y_cells = (lat_max_in - m.latitude) / cell_lat_deg;
                s.set_x(x_cells);
                s.set_y(y_cells);
                s.set_dist(std::max((double)m.distance / meters_per_cell, 1.0));
                s.set_std_dev(std::max(1.0, sigmaMeters / meters_per_cell));
                sensors.push_back(s);
            }

            SourceEstimator estimator;
            auto probMatrix = estimator.computeProbabilityMatrix(sensors, gridW, gridH);

            outGridW = gridW;
            outGridH = gridH;
            out_cell_lat_deg = cell_lat_deg;
            out_cell_lon_deg = cell_lon_deg;
            return probMatrix;
        };

    // --- Bounding box по окну ---
    double lat_min = std::numeric_limits<double>::max();
    double lat_max = std::numeric_limits<double>::lowest();
    double lon_min = std::numeric_limits<double>::max();
    double lon_max = std::numeric_limits<double>::lowest();

    // Используем окно последних WINDOW мер
    std::vector<Measure> window(measures_copy.end() - WINDOW, measures_copy.end());
    for (const auto& m : window) {
        lat_min = std::min(lat_min, m.latitude);
        lat_max = std::max(lat_max, m.latitude);
        lon_min = std::min(lon_min, m.longitude);
        lon_max = std::max(lon_max, m.longitude);
    }

    constexpr double bbox_expand_rel = 0.2;
    constexpr double meters_per_degree_lat = 111320.0;
    double avg_lat_rad_tmp = ((lat_min + lat_max) / 2.0) * M_PI / 180.0;
    double meters_per_degree_lon_tmp = meters_per_degree_lat * std::cos(avg_lat_rad_tmp);

    double lat_expand_sigma_deg = (sigmaMeters * 3.0) / meters_per_degree_lat;
    double lon_expand_sigma_deg = (sigmaMeters * 3.0) / meters_per_degree_lon_tmp;

    double lat_expand = std::max((lat_max - lat_min) * bbox_expand_rel, lat_expand_sigma_deg);
    double lon_expand = std::max((lon_max - lon_min) * bbox_expand_rel, lon_expand_sigma_deg);

    lat_min -= lat_expand; lat_max += lat_expand;
    lon_min -= lon_expand; lon_max += lon_expand;

    unsigned int gridW = 0, gridH = 0;
    double cell_lat_deg = 0.0, cell_lon_deg = 0.0;
    auto probMatrix = buildMatrix(lat_min, lat_max, lon_min, lon_max,
        gridW, gridH, cell_lat_deg, cell_lon_deg);

    if (probMatrix.empty() || probMatrix[0].empty()) {
        notifyError("Ошибка: пустая матрица вероятностей");
        return;
    }

    // --- Поиск максимума ---
    auto findMaxRC = [&](const std::vector<std::vector<double>>& mat) {
        int maxR = 0, maxC = 0;
        double maxV = mat[0][0];
        for (int r = 0; r < (int)mat.size(); ++r)
            for (int c = 0; c < (int)mat[r].size(); ++c)
                if (mat[r][c] > maxV) { maxV = mat[r][c]; maxR = r; maxC = c; }
        return std::tuple<int, int, double>(maxR, maxC, maxV);
        };

    int maxRow = 0, maxCol = 0; double maxVal = 0.0;
    std::tie(maxRow, maxCol, maxVal) = findMaxRC(probMatrix);

    auto isNearBorder = [&](int r, int c, unsigned int W, unsigned int H) {
        int borderCols = std::max(1, (int)(W * 0.05));
        int borderRows = std::max(1, (int)(H * 0.05));
        return (c <= borderCols) || (c >= (int)W - 1 - borderCols) ||
            (r <= borderRows) || (r >= (int)H - 1 - borderRows);
        };

    if (isNearBorder(maxRow, maxCol, gridW, gridH)) {
        double lat_delta = (lat_max - lat_min) * 0.5;
        double lon_delta = (lon_max - lon_min) * 0.5;
        lat_min -= lat_delta; lat_max += lat_delta;
        lon_min -= lon_delta; lon_max += lon_delta;

        unsigned int gridW2 = 0, gridH2 = 0;
        double cell_lat_deg2 = 0.0, cell_lon_deg2 = 0.0;
        auto probMatrix2 = buildMatrix(lat_min, lat_max, lon_min, lon_max,
            gridW2, gridH2, cell_lat_deg2, cell_lon_deg2);
        if (!probMatrix2.empty() && !probMatrix2[0].empty()) {
            probMatrix.swap(probMatrix2);
            gridW = gridW2; gridH = gridH2;
            cell_lat_deg = cell_lat_deg2; cell_lon_deg = cell_lon_deg2;
            std::tie(maxRow, maxCol, maxVal) = findMaxRC(probMatrix);
        }
    }

    double best_lon = lon_min + ((maxCol + 0.5) * cell_lon_deg);
    double best_lat = lat_max - ((maxRow + 0.5) * cell_lat_deg);

    std::vector<float> probZoneFloat;
    probZoneFloat.reserve((size_t)gridW * (size_t)gridH);
    for (auto& row : probMatrix)
        for (double v : row) probZoneFloat.push_back((float)v);

    auto end = std::chrono::steady_clock::now();
    long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_lastCalcTimeMs = elapsedMs;
        m_lastStddevMeters = sigmaMeters;
        m_lastUsedMeasures = measures_copy.size();
    }

    notifyResult(best_lat, best_lon,
        probZoneFloat,
        lat_max, lon_min,
        lat_min, lon_max,
        gridW, gridH);
}

