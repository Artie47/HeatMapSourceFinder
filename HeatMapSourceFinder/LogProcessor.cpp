#include "LogProcessor.h"
#include "PlaceCalcI.h"
#include <iostream>
#include <regex>
#include <fstream>
#include <cmath>
#include <chrono>

LogProcessor::LogProcessor(PlaceCalcI* calc)
    : m_calc(calc)
    , outlier_filter_(30, 3.0, 1.0) // Initialize filter with default parameters
{
}

LogProcessor::~LogProcessor() {
    stop();
}

void LogProcessor::stop() {
    m_stop = true;
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void LogProcessor::set_filter_parameters(size_t window_size, double distance_threshold, double coord_threshold_km) {
    outlier_filter_.set_parameters(window_size, distance_threshold, coord_threshold_km);
}

void LogProcessor::processFile(const std::string& path, ReadMode mode, int intervalSeconds) {
    m_stop = false;

    // Reset filter when starting new processing
    outlier_filter_.reset();

    if (mode == ReadMode::Offline) {
        runOffline(path);
    }
    else {
        runOnline(path, intervalSeconds);
    }
}

bool LogProcessor::loadFromFile(const std::string& filename, const std::string& sensorId) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Ошибка: не удалось открыть файл " << filename << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_carriers.clear();
    m_lastPositions.clear();

    std::string line;
    std::regex pattern_with_id(R"(^\s*\d{2}:\d{2}:\d{2}\.\d{3}\s+(\S+)\.?\s+add\s+measure\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+(\d+).*\(t\s+(\d+)\))");
    std::regex pattern_simple(R"(^\s*\d+\s+add\s+measure\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+(\d+).*\(t\s+(\d+)\))");

    std::string selectedSensorId = sensorId;
    bool sensorIdSet = !sensorId.empty();

    while (std::getline(file, line)) {
        std::smatch match;
        if (std::regex_search(line, match, pattern_with_id)) {
            std::string currentId = match[1].str();
            if (!sensorIdSet) {
                selectedSensorId = currentId;
                sensorIdSet = true;
            }
            if (currentId != selectedSensorId) continue;

            double latitude = std::stod(match[2].str());
            double longitude = std::stod(match[3].str());

            m_lastPositions[currentId] = { latitude, longitude };
            m_carriers.emplace_back(latitude, longitude);
        }
        else if (std::regex_search(line, match, pattern_simple)) {
            if (!sensorIdSet) sensorIdSet = true;

            double latitude = std::stod(match[1].str());
            double longitude = std::stod(match[2].str());

            m_lastPositions[kSingleSensorId] = { latitude, longitude };
            m_carriers.emplace_back(latitude, longitude);
        }
    }

    return sensorIdSet && !m_lastPositions.empty();
}

void LogProcessor::runOffline(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << path << "\n";
        return;
    }

    // Reset filter for new file processing
    outlier_filter_.reset();

    if (m_calc) m_calc->reset();
    m_stop = false;

    const int warmupLines = 30;
    int parsedCount = 0;

    std::string line;
    while (!m_stop && std::getline(file, line)) {
        if (line.empty()) continue;

        bool parsed = parseLine(line);
        if (parsed) {
            if (m_calc) m_calc->calc();
            ++parsedCount;
        }

        if (parsedCount < warmupLines) {
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!m_stop && m_calc) {
        std::cerr << "[LogProcessor] finished file, calling final calc()\n";
        m_calc->calc();
    }
}



void LogProcessor::runOnline(const std::string& path, int intervalSeconds) {
    std::ifstream file(path, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << path << "\n";
        return;
    }

    file.seekg(0, std::ios::end);
    m_lastFilePos = file.tellg();

    while (!m_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(intervalSeconds));

        file.clear();
        file.seekg(m_lastFilePos, std::ios::beg);

        std::string line;
        bool added = false;
        while (std::getline(file, line)) {
            if (parseLine(line)) added = true;
        }

        m_lastFilePos = file.tellg();

        if (added && m_calc) m_calc->calc();
    }
}

bool LogProcessor::parseLine(const std::string& line) {
    static std::regex re_simple(R"(^\s*\d+\s+add\s+measure\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+(\d+).*\(t\s+(\d+)\))");
    static std::regex re_with_id(R"(^\s*\d{2}:\d{2}:\d{2}\.\d{3}\s+(\S+)\.?\s+add\s+measure\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+(\d+).*\(t\s+(\d+)\))");

    std::smatch match;
    if (std::regex_search(line, match, re_with_id)) {
        std::string id = match[1].str();
        double lat = std::stod(match[2].str());
        double lon = std::stod(match[3].str());
        int distance = static_cast<int>(std::lround(std::stod(match[4].str())));
        time_t ts = static_cast<time_t>(std::stoll(match[5].str()));

        // Check for outliers before processing
        if (outlier_filter_.is_outlier(lat, lon, distance)) {
            std::cerr << "[LogProcessor] Outlier filtered: " << lat << ", " << lon << ", " << distance << std::endl;
            return false;
        }

        // Update filter with valid measurement
        outlier_filter_.add_measurement(lat, lon, distance);

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_carriers.emplace_back(lat, lon);
            m_lastPositions[id] = { lat, lon };
        }
        if (m_calc) m_calc->addMeasure(lat, lon, distance, ts);
        return true;
    }
    else if (std::regex_search(line, match, re_simple)) {
        double lat = std::stod(match[1].str());
        double lon = std::stod(match[2].str());
        int distance = static_cast<int>(std::lround(std::stod(match[3].str())));
        time_t ts = static_cast<time_t>(std::stoll(match[4].str()));

        // Check for outliers before processing
        if (outlier_filter_.is_outlier(lat, lon, distance)) {
            std::cerr << "[LogProcessor] Outlier filtered: " << lat << ", " << lon << ", " << distance << std::endl;
            return false;
        }

        // Update filter with valid measurement
        outlier_filter_.add_measurement(lat, lon, distance);

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_carriers.emplace_back(lat, lon);
            m_lastPositions[kSingleSensorId] = { lat, lon };
        }
        if (m_calc) m_calc->addMeasure(lat, lon, distance, ts);
        return true;
    }

    // Log format not recognized
    if (!line.empty()) {
        std::string preview = line.size() > 200 ? line.substr(0, 200) + "..." : line;
        std::cerr << "[LogProcessor::parseLine] unsupported format: '" << preview << "'\n";
    }
    return false;
}

std::vector<std::pair<double, double>> LogProcessor::getCarriers() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_carriers;
}

std::vector<std::pair<double, double>> LogProcessor::getLastCarrierPositions() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_carriers.empty())
        return {};
    return { m_carriers.back() };
}
