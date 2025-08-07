#include "HeatMapSourceFinder.h"
#include "ui_HeatMapSourceFinder.h"
#include "SourceEstimator.h"
#include "HeatMapRenderer.h"
#include "Sensor.h"
#include <QPainter>

HeatMapSourceFinder::HeatMapSourceFinder(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::HeatMapSourceFinderClass)
{
    ui->setupUi(this);

    // Настраиваем heatmapLabel: масштаб и размер
    ui->heatmapLabel->setScaledContents(false);
    ui->heatmapLabel->setMinimumSize(600, 600);
    ui->heatmapLabel->setAlignment(Qt::AlignCenter); // по центру, а не в углу

    // Создание сенсоров
    std::vector<Sensor> sensors = {
         Sensor(1, 1, 2, 3.0),     // Левый нижний угол
         Sensor(1, 9, 2, 3.0),    // Правый нижний угол
         Sensor(8, 8, 2, 5.0)
    };

    // Оценка матрицы вероятностей
    SourceEstimator estimator;
    auto matrix = estimator.computeProbabilityMatrix(sensors, 10, 10);

    // Преобразование в изображение
    HeatMapRenderer renderer;
    QImage heatmap = renderer.matrixToHeatmapImage(matrix);

    int matrixWidth = matrix[0].size();
    int matrixHeight = matrix.size();

    renderer.drawSensorsOnMatrix(heatmap, sensors, matrixWidth, matrixHeight, 50.0f, 50.0f);


    // Установка изображения в label
    ui->heatmapLabel->setPixmap(QPixmap::fromImage(heatmap));
}

HeatMapSourceFinder::~HeatMapSourceFinder()
{
    delete ui;
}
