// HeatMapRenderer.h
#pragma once
#include <vector>
#include <QImage>
#include "Sensor.h"

class HeatMapRenderer {
public:
    HeatMapRenderer();
    static QImage matrixToHeatmapImage(const std::vector<std::vector<double>>& matrix);

    void drawSensorsOnMatrix(QImage& image, const std::vector<Sensor>& sensors, int matrixWidth,
                             int matrixHeight, float fieldWidth, float fieldHeight);
    
    QImage renderCompleteHeatmap(const std::vector<std::vector<double>>& matrix,
        const std::vector<Sensor>& sensors);

private:
    // параметры рендеринга, если понадобятся
};
