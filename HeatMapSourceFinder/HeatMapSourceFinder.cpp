// HeatMapSourceFinder.cpp
#include "HeatMapSourceFinder.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QThread>
#include <QMetaObject>
#include <QTimer>
#include <QVariant>
#include <QDebug>

#include <cmath>
#include <algorithm>
#include <QPointF>

static const char* ONLINE_TIMER_OBJECT_NAME = "onlineTimer";

HeatMapSourceFinder::HeatMapSourceFinder(QWidget* parent)
    : QMainWindow(parent),
    m_logProcessor(nullptr)   // пока nullptr, создаём ниже
{
    // --- Core objects ---
    // Передаём в LogProcessor адрес m_calc как PlaceCalcI*
    m_logProcessor = std::make_unique<LogProcessor>(static_cast<PlaceCalcI*>(&m_calc));

    setFilterParameters(30, 5.0, 3.0);

    // --- UI ---
    setWindowTitle("HeatMap Source Finder");
    resize(1000, 1000);

    QWidget* central = new QWidget(this);
    setCentralWidget(central);

    // Buttons & controls
    m_selectFileBtn = new QPushButton("Выбрать лог", this);
    connect(m_selectFileBtn, &QPushButton::clicked, this, &HeatMapSourceFinder::onSelectFile);

    m_modeCombo = new QComboBox(this);
    m_modeCombo->addItem("Offline", static_cast<int>(ReadMode::Offline));
    //m_modeCombo->addItem("Online", static_cast<int>(ReadMode::Online));

    //m_intervalEdit = new QLineEdit(this);
    //m_intervalEdit->setPlaceholderText("Интервал (сек) для Online");
    //m_intervalEdit->setFixedWidth(120);

    m_sensorIdEdit = new QLineEdit(this);
    m_sensorIdEdit->setPlaceholderText("Sensor ID (опц.)");
    m_sensorIdEdit->setFixedWidth(120);


    m_startBtn = new QPushButton("Запустить", this);
    connect(m_startBtn, &QPushButton::clicked, this, &HeatMapSourceFinder::onStartProcessing);

    m_statusLabel = new QLabel("Файл не выбран", this);
    m_estimateLabel = new QLabel("Estimate: N/A", this);

    m_mapLabel = new QLabel(this);
    m_mapLabel->setMinimumSize(500, 500);
    m_mapLabel->setAlignment(Qt::AlignCenter);
    m_mapLabel->setScaledContents(false);

    // В конструкторе HeatMapSourceFinder
    //m_qualityIndicator = new QProgressBar(this);
    //m_qualityIndicator->setRange(0, 100);
    //m_qualityIndicator->setValue(0); // Начинаем с 0, а не с 100
    //m_qualityIndicator->setFormat("Качество данных: %p%");
    //m_qualityIndicator->setStyleSheet("QProgressBar::chunk { background-color: red; }");

    // Добавим в layout
    

    // Layout
    QHBoxLayout* topLayout = new QHBoxLayout;
    topLayout->addWidget(m_selectFileBtn);
    topLayout->addWidget(m_modeCombo);
    topLayout->addWidget(m_intervalEdit);
    topLayout->addWidget(m_startBtn);
    topLayout->addStretch();
    topLayout->addWidget(m_estimateLabel);
    topLayout->addWidget(m_sensorIdEdit);
    //topLayout->addWidget(m_qualityIndicator);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(topLayout);
    mainLayout->addWidget(m_mapLabel, 1);
    mainLayout->addWidget(m_statusLabel);

    central->setLayout(mainLayout);

    // --- Listener ---
    m_calc.setListener(this);
}


HeatMapSourceFinder::~HeatMapSourceFinder() {
    stopProcessingIfRunning();
}

void HeatMapSourceFinder::stopProcessingIfRunning() {
    if (m_logProcessor) m_logProcessor->stop();

    if (auto t = findChild<QTimer*>(ONLINE_TIMER_OBJECT_NAME)) {
        t->stop();
        t->deleteLater();
    }
}

void HeatMapSourceFinder::onSelectFile() {
    QString file = QFileDialog::getOpenFileName(this, "Выберите лог-файл", QString(), "Log files (*.log *.txt);;All files (*.*)");
    if (!file.isEmpty()) {
        m_selectedFile = file;
        m_statusLabel->setText("Выбран файл: " + file);
    }
}



void HeatMapSourceFinder::onStartProcessing() {
    if (m_selectedFile.isEmpty()) {
        QMessageBox::warning(this, "Ошибка", "Сначала выберите файл лога");
        return;
    }

    stopProcessingIfRunning();

    int modeData = m_modeCombo->currentData().toInt();
    ReadMode mode = static_cast<ReadMode>(modeData);

    //int interval = m_intervalEdit->text().toInt();
    //if (mode == ReadMode::Online && interval <= 0) {
    //    QMessageBox::warning(this, "Ошибка", "Укажите корректный интервал (сек).");
    //    return;
    // }

    m_statusLabel->setText("Обработка запущена...");
    //updateQualityIndicator(0);

    // Рекомендуемая модель потоков:
    // LogProcessor::runOffline / processFile выполняются синхронно в том потоке, где вызваны.
    // Поэтому мы создаём QThread::create и запускаем там — один уровень потоков.
    std::string sensorId = m_sensorIdEdit->text().toStdString();

    if (mode == ReadMode::Offline) {
        bool ok = m_logProcessor->loadFromFile(m_selectedFile.toStdString(), sensorId);
        if (!ok) {
            QMessageBox::warning(this, "Ошибка", "Не удалось распарсить файл или нет измерений.");
            m_statusLabel->setText("Парсинг неудачен");
            return;
        }

        m_calc.setGridSize(90, 90);

        QThread* worker = QThread::create([this, file = m_selectedFile.toStdString()]() {
            this->m_logProcessor->processFile(file, ReadMode::Offline, 0);
            QMetaObject::invokeMethod(this, [this]() {
                this->m_statusLabel->setText("Offline расчёт завершён.");
                }, Qt::QueuedConnection);
            });


        connect(worker, &QThread::finished, worker, &QThread::deleteLater);
        worker->start();
    }
    /*else {
        // Online: load carriers for visualization (doesn't add measures)
        m_logProcessor->loadFromFile(m_selectedFile.toStdString(), sensorId);

        // Запускаем обработку в отдельном QThread — предполагается, что processFile выполняется синхронно
        QThread* worker = QThread::create([this, file = m_selectedFile.toStdString(), interval]() {
            this->m_logProcessor->processFile(file, ReadMode::Online, interval);
            QMetaObject::invokeMethod(this, [this]() {
                this->m_statusLabel->setText("Online режим завершён.");
                }, Qt::QueuedConnection);
            });
        connect(worker, &QThread::finished, worker, &QThread::deleteLater);
        worker->start();

        QTimer* timer = new QTimer(this);
        timer->setObjectName(ONLINE_TIMER_OBJECT_NAME);
        int secondsLeft = interval;
        this->setProperty("secondsLeft", QVariant(secondsLeft));
        connect(timer, &QTimer::timeout, this, [this, interval]() {
            QVariant v = this->property("secondsLeft");
            int sec = v.isValid() ? v.toInt() : interval;
            if (sec > 0) sec--;
            this->setProperty("secondsLeft", QVariant(sec));
            this->m_statusLabel->setText(QString("Online: обновление через %1 сек.").arg(sec));
            }, Qt::UniqueConnection);
        timer->start(1000);
    }*/
}

// handleCalcResultOnGui: гарантированно вызывается в GUI-потоке.
// Объявление этой функции должно быть в HeatMapSourceFinder.h
// ... (все предыдущие includes остаются)
#include <QPointF>

// --- handleCalcResultOnGui: теперь собираем trail из последних 30 точек и передаем в render ---
void HeatMapSourceFinder::handleCalcResultOnGui(
    double latitude,
    double longitude,
    std::vector<float> probZone,
    double zoneLeftTopLatitude,
    double zoneLeftTopLongitude,
    double zoneRightBottomLatitude,
    double zoneRightBottomLongitude,
    unsigned int zoneWidth,
    unsigned int zoneHeight)
{
    try {
        if (m_calc.getLastUsedMeasures() <= 30) {
            // Показываем информационное сообщение
            m_statusLabel->setText(QString("Накоплено %1 из 30 измерений. Ожидаем больше данных...")
                .arg(m_calc.getLastUsedMeasures()));

            // Если прошло много времени, но данных все еще мало - предупреждаем о возможной проблеме
            if (m_calc.getLastUsedMeasures() > 0 && m_calc.getLastUsedMeasures() < 10) {
                m_estimateLabel->setText("Возможны проблемы с данными: слишком мало измерений");
            }

            // Всегда проверяем качество данных, даже если их мало
            //checkDataQuality();
            return;
        }

        if (zoneWidth == 0 || zoneHeight == 0 || probZone.empty()) {
            QMessageBox::warning(this, "Ошибка", "Пустая матрица вероятностей.");
            //checkDataQuality(); // Проверяем качество и в случае ошибки
            return;
        }

        // восстановим матрицу
        std::vector<std::vector<double>> matrix(zoneHeight, std::vector<double>(zoneWidth));
        for (unsigned int y = 0; y < zoneHeight; ++y)
            for (unsigned int x = 0; x < zoneWidth; ++x)
                matrix[y][x] = static_cast<double>(probZone[y * zoneWidth + x]);

        // получим carriers (как раньше)
        std::vector<std::pair<double, double>> carriers;
        if (m_logProcessor) carriers = m_logProcessor->getLastCarrierPositions();

        // --- собрать хвост: последние 30 позиций из LogProcessor::getCarriers() ---
        std::deque<std::pair<double,double>> trailDeque;
        if (m_logProcessor) {
            auto all = m_logProcessor->getCarriers();
            const size_t WANT = 30;
            size_t start = (all.size() > WANT) ? (all.size() - WANT) : 0;
            for (size_t i = start; i < all.size(); ++i) {
                trailDeque.emplace_back(all[i].first, all[i].second);
            }
        }

        // Сохраним также в m_carrierTrail (для совместимости с UI/другими местами)
        m_carrierTrail.clear();
        for (const auto &p : trailDeque) {
            m_carrierTrail.push_back(p);
            if (m_carrierTrail.size() > 30) m_carrierTrail.pop_front();
        }

        // Вызов рендера — передаём trailDeque в renderHeatmapFromMatrix
        renderHeatmapFromMatrix(matrix, carriers,
            zoneLeftTopLatitude, zoneLeftTopLongitude,
            zoneRightBottomLatitude, zoneRightBottomLongitude,
            latitude, longitude, m_calc.getLastStddevMeters(), m_carrierTrail);

        double sigma = m_calc.getLastStddevMeters();
        m_estimateLabel->setText(QString("Estimate: %1, %2 | Measures: %3 | Time: %4 ms | SensorID: %5 | σ≈%6 м")
            .arg(latitude, 0, 'f', 6)
            .arg(longitude, 0, 'f', 6)
            .arg(m_calc.getLastUsedMeasures())
            .arg(m_calc.getLastCalcTimeMs())
            .arg(QString::fromStdString(m_calc.getLastSensorId()))
            .arg(sigma, 0, 'f', 1));

        //checkDataQuality();
    }
    catch (const std::exception& ex) {
        qWarning() << "handleCalcResultOnGui exception:" << ex.what();
        QMessageBox::warning(this, "GUI exception", ex.what());
        //checkDataQuality();
    }
    catch (...) {
        qWarning() << "handleCalcResultOnGui unknown exception";
        QMessageBox::warning(this, "GUI exception", "Неизвестная ошибка в обработке результата.");
        //checkDataQuality();
    }
}


// PlaceCalcListener callbacks
void __stdcall HeatMapSourceFinder::onCalcResult(
    double latitude,
    double longitude,
    float* probabilityZone,
    double zoneLeftTopLatitude,
    double zoneLeftTopLongitude,
    double zoneRightBottomLatitude,
    double zoneRightBottomLongitude,
    unsigned int zoneWidth,
    unsigned int zoneHeight)
{
    // Если вызов не в GUI-потоке — скопировать данные и переслать в GUI
    if (QThread::currentThread() != this->thread()) {
        size_t total = static_cast<size_t>(zoneWidth) * static_cast<size_t>(zoneHeight);
        std::vector<float> copy;
        if (probabilityZone && total > 0) {
            copy.assign(probabilityZone, probabilityZone + total);
        }
        // Передаём move-cопию в лямбду (будет жить до выполнения в GUI-потоке)
        QMetaObject::invokeMethod(this, [this, latitude, longitude,
            zoneLeftTopLatitude, zoneLeftTopLongitude,
            zoneRightBottomLatitude, zoneRightBottomLongitude,
            zoneWidth, zoneHeight, prob = std::move(copy)]() mutable {
                // вызываем обработчик который принимает ownership вектора
                this->handleCalcResultOnGui(latitude, longitude, std::move(prob),
                    zoneLeftTopLatitude, zoneLeftTopLongitude,
                    zoneRightBottomLatitude, zoneRightBottomLongitude,
                    zoneWidth, zoneHeight);
            }, Qt::QueuedConnection);
        return;
    }

    // Если уже GUI-поток — завернём raw pointer в вектор (чтобы единообразно работать дальше)
    size_t total = static_cast<size_t>(zoneWidth) * static_cast<size_t>(zoneHeight);
    std::vector<float> vec;
    if (probabilityZone && total > 0) {
        vec.assign(probabilityZone, probabilityZone + total);
    }
    handleCalcResultOnGui(latitude, longitude, std::move(vec),
        zoneLeftTopLatitude, zoneLeftTopLongitude,
        zoneRightBottomLatitude, zoneRightBottomLongitude,
        zoneWidth, zoneHeight);
}


void __stdcall HeatMapSourceFinder::onError(const char* message) {
    QString msg = QString::fromUtf8(message);
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, msg]() {
            QMessageBox::warning(this, "PlaceCalc error", msg);
            m_statusLabel->clear();
            }, Qt::QueuedConnection);
        return;
    }

    QMessageBox::warning(this, "PlaceCalc error", msg);
    m_statusLabel->clear();
}

// -----------------------------------------------------------------------------
// ЗАМЕНА: renderHeatmapFromMatrix — добавляем gaussian prior от носителей
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
 // REPLACE: HeatMapSourceFinder::renderHeatmapFromMatrix
 // Рисует тепловую карту и справа добавляет панель с метриками:
 // - ширина/высота поля в километрах
 // - ширина/высота одной ячейки в метрах (lat/lon и средняя)
 // - разрешение матрицы (W x H)
// -----------------------------------------------------------------------------
// --- renderHeatmapFromMatrix: подготовим trailCells и передадим в рендерер ---
void HeatMapSourceFinder::renderHeatmapFromMatrix(
    const std::vector<std::vector<double>>& matrix,
    const std::vector<std::pair<double, double>>& carriers,
    double zoneLeftTopLatitude,
    double zoneLeftTopLongitude,
    double zoneRightBottomLatitude,
    double zoneRightBottomLongitude,
    double estLat,
    double estLon,
    double sigmaMeters,
    const std::deque<std::pair<double, double>>& trail)
{
    if (matrix.empty() || matrix[0].empty()) return;

    unsigned int gridH = static_cast<unsigned int>(matrix.size());
    unsigned int gridW = static_cast<unsigned int>(matrix[0].size());

    // Нормализация
    double maxVal = 0.0;
    for (const auto& row : matrix)
        for (double v : row)
            if (v > maxVal) maxVal = v;

    std::vector<std::vector<double>> normalizedMatrix(gridH, std::vector<double>(gridW, 0.0));
    if (maxVal > 0.0) {
        for (unsigned int y = 0; y < gridH; ++y)
            for (unsigned int x = 0; x < gridW; ++x)
                normalizedMatrix[y][x] = matrix[y][x] / maxVal;
    }

    // Преобразуем координаты носителей в координаты сетки
    std::vector<Sensor> sensorsForRender;
    if (!carriers.empty()) {
        double lat_min = zoneRightBottomLatitude;
        double lat_max = zoneLeftTopLatitude;
        double lon_min = zoneLeftTopLongitude;
        double lon_max = zoneRightBottomLongitude;

        double lat_range = lat_max - lat_min;
        double lon_range = lon_max - lon_min;
        if (lat_range <= 0.0) lat_range = 1e-12;
        if (lon_range <= 0.0) lon_range = 1e-12;

        for (const auto& p : carriers) {
            double lat = p.first;
            double lon = p.second;
            double nx = (lon - lon_min) / lon_range;
            double ny = (lat_max - lat) / lat_range; // y=0 вверху
            double fx = std::clamp(nx * (gridW - 1), 0.0, static_cast<double>(gridW - 1));
            double fy = std::clamp(ny * (gridH - 1), 0.0, static_cast<double>(gridH - 1));
            sensorsForRender.emplace_back(fx, fy, 1.0, 0.0);
        }
    }

    // --- преобразуем trail (lat,lon) -> vector<QPointF> с fractional cell coords ---
    std::vector<QPointF> trailCells;
    if (!trail.empty()) {
        double lat_min = zoneRightBottomLatitude;
        double lat_max = zoneLeftTopLatitude;
        double lon_min = zoneLeftTopLongitude;
        double lon_max = zoneRightBottomLongitude;

        double lat_range = lat_max - lat_min;
        double lon_range = lon_max - lon_min;
        if (lat_range <= 0.0) lat_range = 1e-12;
        if (lon_range <= 0.0) lon_range = 1e-12;

        for (const auto& p : trail) {
            double lat = p.first;
            double lon = p.second;
            double nx = (lon - lon_min) / lon_range;
            double ny = (lat_max - lat) / lat_range;
            double fx = std::clamp(nx * (gridW - 1), 0.0, static_cast<double>(gridW - 1));
            double fy = std::clamp(ny * (gridH - 1), 0.0, static_cast<double>(gridH - 1));
            trailCells.emplace_back(fx, fy);
        }
    }

    // Рендерим с передачей trailCells
    HeatMapRenderer renderer;
    QImage heatImage = renderer.renderCompleteHeatmap(normalizedMatrix, sensorsForRender, trailCells);

    // --- Подготовка панели справа (sidebar) ---
    const int sidebarPx = 340;
    int outWidth = heatImage.width() + sidebarPx;
    int outHeight = heatImage.height();

    QImage outImage(outWidth, outHeight, QImage::Format_ARGB32_Premultiplied);
    outImage.fill(Qt::white);

    // Скопируем карту
    {
        QPainter p(&outImage);
        p.drawImage(0, 0, heatImage);
    }

    // Вычисления полей/ячейки (тот же код что и у тебя — оставил без изменений)
    double lat_min = zoneRightBottomLatitude;
    double lat_max = zoneLeftTopLatitude;
    double lon_min = zoneLeftTopLongitude;
    double lon_max = zoneRightBottomLongitude;

    double lat_range_deg = lat_max - lat_min;
    double lon_range_deg = lon_max - lon_min;
    if (lat_range_deg <= 0.0) lat_range_deg = 1e-12;
    if (lon_range_deg <= 0.0) lon_range_deg = 1e-12;

    constexpr double meters_per_deg_lat = 111320.0;
    double avg_lat_rad = ((lat_min + lat_max) / 2.0) * M_PI / 180.0;
    double meters_per_deg_lon = meters_per_deg_lat * std::cos(avg_lat_rad);

    double heightMeters = lat_range_deg * meters_per_deg_lat; // N-S
    double widthMeters = lon_range_deg * meters_per_deg_lon; // E-W

    double cell_lat_deg = lat_range_deg / static_cast<double>(gridH);
    double cell_lon_deg = lon_range_deg / static_cast<double>(gridW);

    double cell_height_m = std::fabs(cell_lat_deg * meters_per_deg_lat);
    double cell_width_m = std::fabs(cell_lon_deg * meters_per_deg_lon);
    double cell_avg_m = (cell_height_m + cell_width_m) * 0.5;

    QStringList lines;
    lines << QString("Поле (ширина E-W): %1 км").arg(QString::number(widthMeters / 1000.0, 'f', 3));
    lines << QString("Поле (высота N-S): %1 км").arg(QString::number(heightMeters / 1000.0, 'f', 3));
    lines << QString("");
    lines << QString("Размер ячейки (ширина): %1 м").arg(QString::number(cell_width_m, 'f', 2));
    lines << QString("Размер ячейки (высота): %1 м").arg(QString::number(cell_height_m, 'f', 2));
    lines << QString("Размер ячейки (средний): %1 м").arg(QString::number(cell_avg_m, 'f', 2));
    lines << QString("");
    lines << QString("Разрешение матрицы: %1 x %2 (W x H)").arg(QString::number(gridW)).arg(QString::number(gridH));
    lines << QString("");
    lines << QString("Sigma (м): %1").arg(QString::number(sigmaMeters, 'f', 1));
    lines << QString("Est (lat,lon): %1, %2").arg(QString::number(estLat, 'f', 6)).arg(QString::number(estLon, 'f', 6));

    {
        QPainter p(&outImage);
        QRect panelRect(heatImage.width(), 0, sidebarPx, outHeight);
        QColor panelBg(245, 245, 245, 230);
        p.fillRect(panelRect, panelBg);

        const int leftMargin = heatImage.width() + 12;
        int y = 12;

        QFont font;
        font.setPointSize(12);
        font.setBold(false);
        p.setFont(font);
        p.setPen(QPen(Qt::black));

        QFont headerFont = font;
        headerFont.setPointSize(14);
        headerFont.setBold(true);
        p.setFont(headerFont);
        p.drawText(QRect(leftMargin, y, sidebarPx - 20, 22), Qt::AlignLeft | Qt::AlignVCenter, QString("Параметры сетки"));
        y += 28;

        p.setFont(font);
        for (const QString& ln : lines) {
            p.drawText(QRect(leftMargin, y, sidebarPx - 22, 18), Qt::AlignLeft | Qt::AlignVCenter, ln);
            y += 18;
        }

        p.setPen(QPen(Qt::gray));
        p.drawLine(leftMargin, y + 6, leftMargin + sidebarPx - 24, y + 6);
    }

    QPixmap pm = QPixmap::fromImage(outImage);
    QSize target = m_mapLabel->size();
    if (target.width() > 0 && target.height() > 0) {
        QPixmap scaled = pm.scaled(target, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        m_mapLabel->setPixmap(scaled);
    }
    else {
        m_mapLabel->setPixmap(pm);
    }
}


void HeatMapSourceFinder::setFilterParameters(size_t window_size, double distance_threshold, double coord_threshold_km) {
    if (m_logProcessor) {
        m_logProcessor->set_filter_parameters(window_size, distance_threshold, coord_threshold_km);
    }
}

void HeatMapSourceFinder::checkDataQuality() {
    qDebug() << "=== checkDataQuality called ===";

    if (!m_logProcessor) {
        qDebug() << "No log processor";
        updateQualityIndicator(0);
        return;
    }

    auto carriers = m_logProcessor->getCarriers();
    qDebug() << "Number of carriers:" << carriers.size();

    if (carriers.size() < 5) {
        qDebug() << "Not enough carriers for quality check";
        updateQualityIndicator(0);
        return;
    }

    // Анализируем данные на предмет аномалий
    double total_distance = 0;
    std::vector<double> distances;
    int outlier_count = 0;

    for (size_t i = 1; i < carriers.size(); i++) {
        double dist = RealTimeOutlierFilter::haversine_distance(
            carriers[i - 1].first, carriers[i - 1].second,
            carriers[i].first, carriers[i].second);
        distances.push_back(dist);
        total_distance += dist;
        qDebug() << "Distance between point" << i - 1 << "and" << i << ":" << dist << "km";
    }

    if (distances.empty()) {
        qDebug() << "No distances calculated";
        updateQualityIndicator(0);
        return;
    }

    // Вычисляем среднее и стандартное отклонение
    double avg_distance = total_distance / distances.size();
    double sq_sum = 0;
    for (double d : distances) {
        sq_sum += (d - avg_distance) * (d - avg_distance);
    }
    double std_dev = std::sqrt(sq_sum / distances.size());

    qDebug() << "Average distance:" << avg_distance << "km, Standard deviation:" << std_dev << "km";

    // Установим минимальное стандартное отклонение в 10 метров (0.01 км)
    double min_std_dev = 0.01; // 10 метров в км
    double effective_std_dev = std::max(std_dev, min_std_dev);
    qDebug() << "Effective standard deviation:" << effective_std_dev << "km";

    // Считаем выбросы (более 3 стандартных отклонений от среднего)
    for (double d : distances) {
        if (std::abs(d - avg_distance) > 3 * effective_std_dev) {
            outlier_count++;
            qDebug() << "Outlier found:" << d << "km (threshold:" << 3 * effective_std_dev << "km)";
        }
    }

    // Вычисляем процент выбросов
    double outlier_percentage = (double)outlier_count / distances.size() * 100;
    qDebug() << "Outlier percentage:" << outlier_percentage << "%";

    // Вычисляем общее качество (100% - процент выбросов, но не менее 0)
    int quality = std::max(0, 100 - (int)outlier_percentage);

    // Дополнительные проверки
    if (avg_distance < 0.001 && carriers.size() > 10) {
        // Минимальное перемещение при многих измерениях
        quality = std::max(0, quality - 30);
        qDebug() << "Low movement penalty applied. New quality:" << quality;
    }

    qDebug() << "Final quality score:" << quality;
    updateQualityIndicator(quality);

    // Обновляем статус в зависимости от качества
    if (quality < 30) {
        m_statusLabel->setText("Низкое качество данных: много выбросов (" +
            QString::number(outlier_percentage, 'f', 1) + "%)");
    }
    else if (quality < 70) {
        m_statusLabel->setText("Среднее качество данных: " +
            QString::number(outlier_percentage, 'f', 1) + "% выбросов");
    }
    else {
        m_statusLabel->setText("Высокое качество данных: " +
            QString::number(outlier_percentage, 'f', 1) + "% выбросов");
    }

    qDebug() << "=== checkDataQuality finished ===";
}

void HeatMapSourceFinder::updateQualityIndicator(int quality) {
    qDebug() << "Updating quality indicator to:" << quality;

    // Принудительно ограничиваем значение от 0 до 100
    int clampedQuality = std::max(0, std::min(100, quality));

    if (clampedQuality != quality) {
        qDebug() << "Clamped quality from" << quality << "to" << clampedQuality;
    }

    m_qualityIndicator->setValue(clampedQuality);

    // Меняем цвет в зависимости от качества
    if (clampedQuality > 70) {
        m_qualityIndicator->setStyleSheet("QProgressBar::chunk { background-color: green; }");
    }
    else if (clampedQuality > 40) {
        m_qualityIndicator->setStyleSheet("QProgressBar::chunk { background-color: orange; }");
    }
    else {
        m_qualityIndicator->setStyleSheet("QProgressBar::chunk { background-color: red; }");
    }
}
