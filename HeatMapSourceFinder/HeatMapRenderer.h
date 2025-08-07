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

private:
    // параметры рендеринга, если понадобятся
};
