#include "HeatMapSourceFinder.h"
#include "ui_HeatMapSourceFinder.h"
#include "SourceEstimator.h"
#include "HeatMapRenderer.h"

#include <QInputDialog>
#include <QMessageBox>
#include <QPainter>
#include <QRandomGenerator>
#include <algorithm>
#include <cmath>
#include <limits>

HeatMapSourceFinder::HeatMapSourceFinder(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::HeatMapSourceFinderClass)
{
    ui->setupUi(this);

    // Диалог: размеры матрицы
    bool okW = false, okH = false;
    matrixWidth = QInputDialog::getInt(this, "Размер матрицы", "Введите ширину матрицы:", 30, 1, 2000, 1, &okW);
    matrixHeight = QInputDialog::getInt(this, "Размер матрицы", "Введите высоту матрицы:", 30, 1, 2000, 1, &okH);
    if (!okW || !okH) {
        QApplication::quit();
        return;
    }

    // Панель справа (размеры и кнопки)
    QVBoxLayout* controlsLayout = new QVBoxLayout;
    controlsLayout->addWidget(new QLabel("Размер матрицы:"));

    matrixWidthEdit = new QLineEdit(QString::number(matrixWidth));
    matrixHeightEdit = new QLineEdit(QString::number(matrixHeight));
    matrixWidthEdit->setFixedWidth(100);
    matrixHeightEdit->setFixedWidth(100);
    controlsLayout->addWidget(matrixWidthEdit);
    controlsLayout->addWidget(matrixHeightEdit);

    // Build button — должен быть первым
    buildButton = new QPushButton("Построить карту");
    controlsLayout->addWidget(buildButton);
    connect(buildButton, &QPushButton::clicked, this, &HeatMapSourceFinder::buildHeatmapFromInputs);

    // Time button — неактивна до первого построения карты
    timeButton = new QPushButton("Включить время");
    timeButton->setEnabled(false); // не доступна до построения карты
    controlsLayout->addWidget(timeButton);
    connect(timeButton, &QPushButton::clicked, this, &HeatMapSourceFinder::toggleSimulation);

    // место для оценки местоположения
    estimateLabel = new QLabel("Estimate: N/A");
    controlsLayout->addWidget(estimateLabel);

    controlsLayout->addStretch();

    //Левая панель (список датчиков)
    sensorInputLayout = new QVBoxLayout;

    // Заголовки столбцов (под картой)
    QHBoxLayout* headerRow = new QHBoxLayout;
    headerRow->addWidget(new QLabel("№"));
    headerRow->addWidget(new QLabel("X"));
    headerRow->addWidget(new QLabel("Y"));
    headerRow->addWidget(new QLabel("std_dev"));
    headerRow->addWidget(new QLabel("dist"));
    headerRow->addSpacing(30);
    sensorInputLayout->addLayout(headerRow);

 
    addSensorButton = new QPushButton("Добавить датчик");
    connect(addSensorButton, &QPushButton::clicked, this, [this]() { addSensorRow(true); });
    sensorInputLayout->addWidget(addSensorButton);

    // Основной layout
    QHBoxLayout* mainInputLayout = new QHBoxLayout;
    mainInputLayout->addLayout(sensorInputLayout, 3);
    mainInputLayout->addLayout(controlsLayout, 1);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(ui->heatmapLabel);
    mainLayout->addLayout(mainInputLayout);

    QWidget* centralWidget = new QWidget;
    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);

    // Рисуем начальную серую сетку
    const int canvasSize = 500;
    QImage empty(canvasSize, canvasSize, QImage::Format_ARGB32);
    empty.fill(Qt::gray);
    QPainter painter(&empty);
    painter.setPen(QPen(Qt::black));
    float cellW = static_cast<float>(canvasSize) / static_cast<float>(matrixWidth);
    float cellH = static_cast<float>(canvasSize) / static_cast<float>(matrixHeight);
    for (int x = 0; x <= matrixWidth; ++x) painter.drawLine(std::lround(x * cellW), 0, std::lround(x * cellW), canvasSize);
    for (int y = 0; y <= matrixHeight; ++y) painter.drawLine(0, std::lround(y * cellH), canvasSize, std::lround(y * cellH));
    painter.end();
    ui->heatmapLabel->setPixmap(QPixmap::fromImage(empty));
    ui->heatmapLabel->setMinimumSize(canvasSize, canvasSize);

    // Добавляем по умолчанию пустую строку датчика
    addSensorRow(true);

    // Таймер симуляции
    simulationTimer = new QTimer(this);
    connect(simulationTimer, &QTimer::timeout, this, &HeatMapSourceFinder::simulationStep);
    simulationRunning = false;

    // init smoothing
    lastEstimateX = std::numeric_limits<double>::quiet_NaN();
    lastEstimateY = std::numeric_limits<double>::quiet_NaN();
}

HeatMapSourceFinder::~HeatMapSourceFinder()
{
    delete ui;
}

void HeatMapSourceFinder::addSensorRow(bool insertBeforeButton)
{
    SensorRow sr;
    sr.rowWidget = new QWidget;
    QHBoxLayout* rowLayout = new QHBoxLayout(sr.rowWidget);
    rowLayout->setContentsMargins(0, 0, 0, 0);

    sr.indexLabel = new QLabel(QString::number(sensorRows.size() + 1));
    sr.xEdit = new QLineEdit;
    sr.yEdit = new QLineEdit;
    sr.stdEdit = new QLineEdit;
    sr.distEdit = new QLineEdit;
    sr.removeBtn = new QPushButton("❌");
    sr.removeBtn->setFixedWidth(30);

    sr.xEdit->setPlaceholderText("X");
    sr.yEdit->setPlaceholderText("Y");
    sr.stdEdit->setPlaceholderText("std_dev");
    sr.distEdit->setPlaceholderText("dist");

    rowLayout->addWidget(sr.indexLabel);
    rowLayout->addWidget(sr.xEdit);
    rowLayout->addWidget(sr.yEdit);
    rowLayout->addWidget(sr.stdEdit);
    rowLayout->addWidget(sr.distEdit);
    rowLayout->addWidget(sr.removeBtn);

    if (insertBeforeButton) {
        int pos = sensorInputLayout->count() - 1; // перед кнопкой addSensorButton
        if (pos < 1) pos = 1;
        sensorInputLayout->insertWidget(pos, sr.rowWidget);
    }
    else {
        sensorInputLayout->addWidget(sr.rowWidget);
    }

    sensorRows.push_back(sr);

    // подключаем удаление — удаляем по rowWidget
    QWidget* savedRowWidget = sensorRows.back().rowWidget;
    connect(sensorRows.back().removeBtn, &QPushButton::clicked, this, [this, savedRowWidget]() {
        int idx = -1;
        for (int i = 0; i < static_cast<int>(sensorRows.size()); ++i) {
            if (sensorRows[i].rowWidget == savedRowWidget) { idx = i; break; }
        }
        if (idx == -1) return;
        QWidget* w = sensorRows[idx].rowWidget;
        if (w) { w->hide(); w->setParent(nullptr); w->deleteLater(); }
        sensorRows.erase(sensorRows.begin() + idx);
        renumberSensorLabels();
        });
}

void HeatMapSourceFinder::renumberSensorLabels()
{
    for (int i = 0; i < static_cast<int>(sensorRows.size()); ++i) {
        if (sensorRows[i].indexLabel) sensorRows[i].indexLabel->setText(QString::number(i + 1));
    }
}

void HeatMapSourceFinder::buildHeatmapFromInputs()
{
    updateHeatmap();
}

void HeatMapSourceFinder::updateHeatmap()
{
    bool okW = false, okH = false;
    int width = matrixWidthEdit->text().toInt(&okW);
    int height = matrixHeightEdit->text().toInt(&okH);
    if (!okW || !okH || width <= 0 || height <= 0) {
        QMessageBox::warning(this, "Ошибка", "Неверные размеры матрицы.");
        return;
    }

    // Собираем сенсоры
    std::vector<Sensor> sensors;
    sensors.reserve(sensorRows.size());
    for (auto& sr : sensorRows) {
        bool okX = false, okY = false, okS = false, okD = false;
        int x = sr.xEdit->text().toInt(&okX);
        int y = sr.yEdit->text().toInt(&okY);
        double stddev = sr.stdEdit->text().toDouble(&okS);
        double dist = sr.distEdit->text().toDouble(&okD);
        if (!(okX && okY && okS && okD)) {
            QMessageBox::warning(this, "Ошибка", "Некорректный ввод в полях датчиков.");
            return;
        }
        if (x < 0 || x >= width || y < 0 || y >= height) {
            QMessageBox::warning(this, "Ошибка", "Координаты сенсора выходят за пределы матрицы.");
            return;
        }
        sensors.emplace_back(x, y, stddev, dist);
    }

    // Получаем матрицу
    SourceEstimator estimator;
    auto matrix = estimator.computeProbabilityMatrix(sensors, width, height);

    // Рендерим полную картинку (легенда и датчики)
    HeatMapRenderer renderer;
    QImage heatmap = renderer.renderCompleteHeatmap(matrix, sensors);

    ui->heatmapLabel->setPixmap(QPixmap::fromImage(heatmap));
    ui->heatmapLabel->setMinimumSize(heatmap.size());

    // Включаем кнопку времени
    if (!timeButton->isEnabled()) timeButton->setEnabled(true);

    // Оцениваем наиболее вероятную точку
    auto est = estimator.estimateMostLikelyLocation(matrix, 0.0, static_cast<double>(width), 0.0, static_cast<double>(height));
    double estX = est.first + 0.5;
    double estY = est.second + 0.5;

    // EMA сглаживание
    if (!std::isfinite(lastEstimateX) || !std::isfinite(lastEstimateY)) {
        lastEstimateX = estX;
        lastEstimateY = estY;
    }
    else {
        lastEstimateX = (1.0 - estimateSmoothAlpha) * lastEstimateX + estimateSmoothAlpha * estX;
        lastEstimateY = (1.0 - estimateSmoothAlpha) * lastEstimateY + estimateSmoothAlpha * estY;
    }

    estimateLabel->setText(QString("Estimate: X=%1, Y=%2")
        .arg(lastEstimateX, 0, 'f', 2)
        .arg(lastEstimateY, 0, 'f', 2));
}

void HeatMapSourceFinder::toggleSimulation()
{
    simulationRunning = !simulationRunning;

    // скрываем/показываем кнопки добавления/построения
    addSensorButton->setVisible(!simulationRunning);
    buildButton->setVisible(!simulationRunning);

    // блокируем/разблокируем редактирование и кнопки удаления
    for (auto& sr : sensorRows) {
        if (sr.xEdit) sr.xEdit->setEnabled(!simulationRunning);
        if (sr.yEdit) sr.yEdit->setEnabled(!simulationRunning);
        if (sr.stdEdit) sr.stdEdit->setEnabled(!simulationRunning);
        if (sr.distEdit) sr.distEdit->setEnabled(!simulationRunning);
        if (sr.removeBtn) sr.removeBtn->setEnabled(!simulationRunning);
    }

    if (simulationRunning) {
        // Инициализируем состояние источника: ставим в центр поля (или можно рандом)
        bool okW = false, okH = false;
        int w = matrixWidthEdit->text().toInt(&okW);
        int h = matrixHeightEdit->text().toInt(&okH);
        if (!okW || !okH) { w = matrixWidth; h = matrixHeight; }

        sourceX = (w - 1) / 2.0;
        sourceY = (h - 1) / 2.0;

        // даём случайные начальные скорости (небольшие)
        double r1 = QRandomGenerator::global()->bounded(1.0);
        double r2 = QRandomGenerator::global()->bounded(1.0);
        velX = (r1 * 2.0 - 1.0) * (simMaxVel * 0.2);
        velY = (r2 * 2.0 - 1.0) * (simMaxVel * 0.2);
        accX = accY = 0.0;

        timeButton->setText("Выключить время");
        simulationTimer->start(simulationIntervalMs);
    }
    else {
        timeButton->setText("Включить время");
        simulationTimer->stop();

        // сбросим скорость/ускорение
        velX = velY = accX = accY = 0.0;
    }
}


void HeatMapSourceFinder::simulationStep()
{
    // dt в секундах
    double dt = static_cast<double>(simulationIntervalMs) / 1000.0;

    // читаем границы поля (матрицы)
    bool okW = false, okH = false;
    int w = matrixWidthEdit->text().toInt(&okW);
    int h = matrixHeightEdit->text().toInt(&okH);
    if (!okW || !okH) { w = matrixWidth; h = matrixHeight; }

    // обновляем ускорение, скорость и положение источника
    double ax = (QRandomGenerator::global()->generateDouble() * 2.0 - 1.0) * simMaxAccel;
    double ay = (QRandomGenerator::global()->generateDouble() * 2.0 - 1.0) * simMaxAccel;
    accX = ax;
    accY = ay;

    velX += accX * dt;
    velY += accY * dt;

    // ограничиваем скорость
    velX = std::clamp(velX, -simMaxVel, simMaxVel);
    velY = std::clamp(velY, -simMaxVel, simMaxVel);

    // обновляем позицию
    sourceX += velX * dt;
    sourceY += velY * dt;

    // держим источник внутри поля [0, w-1] x [0, h-1]
    if (sourceX < 0.0) { sourceX = 0.0; velX = -velX * 0.5; } // отскок с демпфированием
    if (sourceX > (w - 1)) { sourceX = (w - 1); velX = -velX * 0.5; }
    if (sourceY < 0.0) { sourceY = 0.0; velY = -velY * 0.5; }
    if (sourceY > (h - 1)) { sourceY = (h - 1); velY = -velY * 0.5; }

    // Теперь для каждого сенсора вычисляем истинное расстояние до источника
    for (auto& sr : sensorRows) {
        bool okx = false, oky = false;
        int sx = sr.xEdit->text().toInt(&okx);
        int sy = sr.yEdit->text().toInt(&oky);
        if (!okx || !oky) continue; // если поле пустое — пропускаем

        double d = std::hypot(sourceX - static_cast<double>(sx), sourceY - static_cast<double>(sy));

        // можно ещё добавить шум измерения, например:
        // double noise = (QRandomGenerator::global()->generateDouble() - 0.5) * noiseLevel;
        // d += noise;

        sr.distEdit->setText(QString::number(d, 'f', 2));
    }

    // Обновляем карту (и estimateLabel внутри updateHeatmap)
    updateHeatmap();
}
