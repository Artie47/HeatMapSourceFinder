#pragma once
#include <vector>
#include <QImage>
#include <QPointF>
#include "Sensor.h"

class HeatMapRenderer {
public:
    HeatMapRenderer();
    static QImage matrixToHeatmapImage(const std::vector<std::vector<double>>& matrix);

    void drawSensorsOnMatrix(QImage& image, const std::vector<Sensor>& sensors, int matrixWidth,
        int matrixHeight, float fieldWidth, float fieldHeight);

    // Добавили опциональный параметр trailCells (в координатах ячеек, дробные индексы)
    QImage renderCompleteHeatmap(const std::vector<std::vector<double>>& matrix,
        const std::vector<Sensor>& sensors,
        const std::vector<QPointF>& trailCells = std::vector<QPointF>());

private:
    // параметры рендеринга, если понадобятся
};
