// HeatMapRenderer.cpp
#include "HeatMapRenderer.h"
#include <QColor>
#include <algorithm>
#include <QPainter>
#include <QImage>
#include <QPen>
#include <QBrush>
#include "Sensor.h"

HeatMapRenderer::HeatMapRenderer() {}

QImage HeatMapRenderer::matrixToHeatmapImage(const std::vector<std::vector<double>>& matrix) {
    const int imgSize = 500;
    const int rows = static_cast<int>(matrix.size());
    const int cols = static_cast<int>(matrix[0].size());

    QImage image(imgSize, imgSize, QImage::Format_RGB32);
    QPainter painter(&image);

    // Нормализация значений от 0 до 1
    double minVal = matrix[0][0], maxVal = matrix[0][0];
    for (const auto& row : matrix)
        for (double val : row) {
            minVal = std::min(minVal, val);
            maxVal = std::max(maxVal, val);
        }

    if (minVal == maxVal) maxVal += 1.0; // чтобы избежать деления на 0

    float cellWidth = imgSize / static_cast<float>(cols);
    float cellHeight = imgSize / static_cast<float>(rows);

    // Отрисовка клеток с плавным градиентом: синий → жёлтый → красный
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            double normVal = (matrix[y][x] - minVal) / (maxVal - minVal);

            QColor color;
            if (normVal < 0.5) {
                // Интерполяция между синим (0,0,255) и жёлтым (255,255,0)
                float t = normVal * 2.0f;
                int r = static_cast<int>(t * 255);
                int g = static_cast<int>(t * 255);
                int b = static_cast<int>(255 * (1 - t));
                color = QColor(r, g, b);
            }
            else {
                // Интерполяция между жёлтым (255,255,0) и красным (255,0,0)
                float t = (normVal - 0.5f) * 2.0f;
                int r = 255;
                int g = static_cast<int>(255 * (1 - t));
                int b = 0;
                color = QColor(r, g, b);
            }

            QRectF cellRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
            painter.fillRect(cellRect, color);
        }
    }

    // Чёрная сетка
    painter.setPen(QPen(Qt::black));
    for (int y = 0; y <= rows; ++y)
        painter.drawLine(0, y * cellHeight, imgSize, y * cellHeight);
    for (int x = 0; x <= cols; ++x)
        painter.drawLine(x * cellWidth, 0, x * cellWidth, imgSize);

    return image;
}



void HeatMapRenderer::drawSensorsOnMatrix(QImage& image, const std::vector<Sensor>& sensors, int matrixWidth, int matrixHeight, float fieldWidth, float fieldHeight) {
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, false); // без сглаживания, чтобы чёткие клетки были

    int pixelWidth = image.width();
    int pixelHeight = image.height();

    float cellWidth = pixelWidth / static_cast<float>(matrixWidth);
    float cellHeight = pixelHeight / static_cast<float>(matrixHeight);

    QPen pen(Qt::gray);
    pen.setWidth(1);
    painter.setPen(pen);
    painter.setBrush(Qt::gray); // Цвет датчика (можно изменить при желании)

    for (const Sensor& sensor : sensors) {
        int matrixX = sensor.get_x();
        int matrixY = sensor.get_y();

        // Верхний левый угол клетки
        float x = matrixX * cellWidth;
        float y = matrixY * cellHeight;

        // Прямоугольник размера ячейки
        QRectF rect(x, y, cellWidth, cellHeight);
        painter.drawRect(rect);
    }

    painter.end();
}



