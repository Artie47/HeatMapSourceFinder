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
    painter.setRenderHint(QPainter::Antialiasing, true); // включаем сглаживание для круга

    int pixelWidth = image.width();
    int pixelHeight = image.height();

    float cellWidth = pixelWidth / static_cast<float>(matrixWidth);
    float cellHeight = pixelHeight / static_cast<float>(matrixHeight);

    QPen pen(Qt::black);
    pen.setWidth(2);
    painter.setPen(pen);
    painter.setBrush(Qt::white); // Белая заливка круга

    for (const Sensor& sensor : sensors) {
        int matrixX = sensor.get_x();
        int matrixY = sensor.get_y();

        // Центр ячейки в пикселях
        float cx = matrixX * cellWidth + cellWidth / 2.0f;
        float cy = matrixY * cellHeight + cellHeight / 2.0f;

        // Радиус круга — вписанный в ячейку (минимум из ширины и высоты / 2)
        float radius = std::min(cellWidth, cellHeight) / 2.0f;

        painter.drawEllipse(QPointF(cx, cy), radius, radius);
    }


    painter.end();
}

QImage HeatMapRenderer::renderCompleteHeatmap(const std::vector<std::vector<double>>& matrix,
    const std::vector<Sensor>& sensors) {
    const int heatmapSize = 500;
    const int legendWidth = 80;
    const int footerHeight = 10; // увеличено для полей

    const int fullWidth = heatmapSize + legendWidth + 20; // небольшой отступ справа
    const int fullHeight = heatmapSize + footerHeight + 20; // отступы сверху/снизу

    QImage finalImage(fullWidth, fullHeight, QImage::Format_RGB32);
    finalImage.fill(Qt::white);

    QPainter painter(&finalImage);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // --- 1. Тепловая карта ---
    QImage heatmapImage = matrixToHeatmapImage(matrix);
    painter.drawImage(0, 0, heatmapImage);

    // Нарисовать датчики поверх
    int matrixWidth = matrix[0].size();
    int matrixHeight = matrix.size();
    drawSensorsOnMatrix(heatmapImage, sensors, matrixWidth, matrixHeight, 1.0f, 1.0f);
    painter.drawImage(0, 0, heatmapImage);

    // --- 2. Легенда справа ---
    const int legendX = heatmapSize + 10;
    const int legendY = 10;
    const int legendHeight = heatmapSize - 20;
    const int legendBarWidth = 20;

    QLinearGradient gradient(legendX, legendY, legendX, legendY + legendHeight);
    gradient.setColorAt(1.0, QColor::fromHsvF(0.0, 1.0, 1.0));    // красный
    gradient.setColorAt(0.5, QColor::fromHsvF(0.16, 1.0, 1.0));   // жёлтый
    gradient.setColorAt(0.0, QColor::fromHsvF(0.66, 1.0, 1.0));   // синий

    painter.setBrush(gradient);
    painter.setPen(Qt::black);
    painter.drawRect(legendX, legendY, legendBarWidth, legendHeight);

    // Подписи вероятностей
    QFont font = painter.font();
    font.setPointSize(8);
    painter.setFont(font);

    int steps = 5;
    for (int i = 0; i <= steps; ++i) {
        double p = 1.0 - static_cast<double>(i) / steps;
        int y = legendY + static_cast<int>(p * legendHeight);
        QString label = QString::number(p, 'f', 2);
        painter.drawText(legendX + legendBarWidth + 5, y + 4, label);
    }

    painter.end();
    return finalImage;
}