#pragma once

#include <QtWidgets/QMainWindow>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <memory>

#include "PlaceCalcI.h"
#include "PlaceCalcImpl.h"
#include "LogProcessor.h"
#include "HeatMapRenderer.h"
#include "Sensor.h"
#include <deque>
#include <QProgressBar>

class HeatMapSourceFinder : public QMainWindow, public PlaceCalcListener {
    Q_OBJECT

public:
    explicit HeatMapSourceFinder(QWidget* parent = nullptr);
    ~HeatMapSourceFinder() override;

    // PlaceCalcListener
    void __stdcall onCalcResult(
        double latitude,
        double longitude,
        float* probabilityZone,
        double zoneLeftTopLatitude,
        double zoneLeftTopLongitude,
        double zoneRightBottomLatitude,
        double zoneRightBottomLongitude,
        unsigned int zoneWidth,
        unsigned int zoneHeight
    ) override;

    void __stdcall onError(const char* message) override;
    void setFilterParameters(size_t window_size, double distance_threshold, double coord_threshold_km);

    void checkDataQuality();

private slots:
    void onSelectFile();
    void updateQualityIndicator(int quality);
    void onStartProcessing();

    void handleCalcResultOnGui(double latitude, double longitude, std::vector<float> probZone, double zoneLeftTopLatitude, double zoneLeftTopLongitude, double zoneRightBottomLatitude, double zoneRightBottomLongitude, unsigned int zoneWidth, unsigned int zoneHeight);

private:
    // UI
    QLabel* m_mapLabel = nullptr;
    QLabel* m_statusLabel = nullptr;
    QLabel* m_estimateLabel = nullptr;
    QPushButton* m_selectFileBtn = nullptr;
    QPushButton* m_startBtn = nullptr;
    QComboBox* m_modeCombo = nullptr;
    QLineEdit* m_intervalEdit = nullptr;
	QLineEdit* m_sensorIdEdit = nullptr;
    QProgressBar* m_qualityIndicator = nullptr;
    // state
    QString m_selectedFile;
    std::deque<std::pair<double, double>> m_carrierTrail;

    // core
    PlaceCalcImpl m_calc;
    std::unique_ptr<LogProcessor> m_logProcessor; // use unique_ptr to construct with &m_calc

    // helper
    void stopProcessingIfRunning();
    void renderHeatmapFromMatrix(
        const std::vector<std::vector<double>>& matrix,
        const std::vector<std::pair<double, double>>& carriers,
        double zoneLeftTopLatitude,
        double zoneLeftTopLongitude,
        double zoneRightBottomLatitude,
        double zoneRightBottomLongitude,
        double estLat,
        double estLon,
        double sigmaMeters,
        const std::deque<std::pair<double, double>>& trail);

};
