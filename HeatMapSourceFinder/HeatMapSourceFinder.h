#pragma once

#include <QMainWindow>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include <QLabel>
#include <vector>

#include "Sensor.h"

QT_BEGIN_NAMESPACE
namespace Ui { class HeatMapSourceFinderClass; }
QT_END_NAMESPACE

class HeatMapSourceFinder : public QMainWindow
{
    Q_OBJECT

public:
    explicit HeatMapSourceFinder(QWidget* parent = nullptr);
    ~HeatMapSourceFinder() override;

private slots:
    void addSensorRow(bool insertBeforeButton = true);
    void renumberSensorLabels();
    void updateHeatmap();
    void toggleSimulation();
    void simulationStep();
    void buildHeatmapFromInputs(); 

private:
    Ui::HeatMapSourceFinderClass* ui;

    struct SensorRow {
        QWidget* rowWidget = nullptr;
        QLabel* indexLabel = nullptr;
        QLineEdit* xEdit = nullptr;
        QLineEdit* yEdit = nullptr;
        QLineEdit* stdEdit = nullptr;
        QLineEdit* distEdit = nullptr;
        QPushButton* removeBtn = nullptr;
        double vel = 0.0; 
        double acc = 0.0; 
    };

    std::vector<SensorRow> sensorRows;

    QVBoxLayout* sensorInputLayout = nullptr;
    QPushButton* addSensorButton = nullptr;
    QPushButton* buildButton = nullptr;
    QPushButton* timeButton = nullptr;
    QLabel* estimateLabel = nullptr;

    QLineEdit* matrixWidthEdit = nullptr;
    QLineEdit* matrixHeightEdit = nullptr;
    int matrixWidth = 30;
    int matrixHeight = 30;

    QTimer* simulationTimer = nullptr;
    bool simulationRunning = false;

    // simulation params

    double estimateSmoothAlpha = 0.25; // EMA smoothing factor for estimated location
    double lastEstimateX = std::numeric_limits<double>::quiet_NaN();
    double lastEstimateY = std::numeric_limits<double>::quiet_NaN();

    // состояние движущегося источника (координаты в пространстве матрицы)
    double sourceX = 0.0;
    double sourceY = 0.0;
    double velX = 0.0;
    double velY = 0.0;
    double accX = 0.0;
    double accY = 0.0;

    // параметры симуляции движения источника
    double simMaxAccel = 1.0;   // макс абсолютное ускорение (ед./с^2)
    double simMaxVel = 5.0;     // макс скорость (ед./с)
    int simulationIntervalMs = 250; // шаг таймера (мс)

};
