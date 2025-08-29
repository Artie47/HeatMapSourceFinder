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
    const int imgSize = 1000;

    if (matrix.empty() || matrix[0].empty()) {
        QImage empty(imgSize, imgSize, QImage::Format_RGB32);
        empty.fill(Qt::gray);
        return empty;
    }

    const int rows = static_cast<int>(matrix.size());
    const int cols = static_cast<int>(matrix[0].size());

    QImage image(imgSize, imgSize, QImage::Format_RGB32);
    image.fill(Qt::white);

    // --- min/max ---
    double minVal = matrix[0][0], maxVal = matrix[0][0];
    for (const auto& row : matrix) {
        for (double v : row) {
            if (v < minVal) minVal = v;
            if (v > maxVal) maxVal = v;
        }
    }
    if (minVal == maxVal) {
        // все значения одинаковые: пусть всё станет серым (иначе деление на 0)
        maxVal = minVal + 1.0;
    }

    // --- Нормализованные значения для порога ---
    std::vector<double> normVals;     normVals.reserve(rows * cols);
    std::vector<double> nzNormVals;   nzNormVals.reserve(rows * cols);
    for (const auto& row : matrix) {
        for (double v : row) {
            double n = (v - minVal) / (maxVal - minVal);
            if (n < 0.0) n = 0.0;
            if (n > 1.0) n = 1.0;
            normVals.push_back(n);
            if (n > 0.0) nzNormVals.push_back(n); // игнорируем чистые нули
        }
    }

    // --- Порог по перцентилю среди ненулевых ---
    const double kVisPercentile = 10.0;     // 10-й перцентиль
    const double kMinThreshold = 0.05;     // нижний пол (5%)
    const double kFallback = 0.10;     // запасной порог, если все нули

    double threshold = kFallback;
    if (!nzNormVals.empty()) {
        std::sort(nzNormVals.begin(), nzNormVals.end());
        size_t idx = static_cast<size_t>(std::floor((kVisPercentile / 100.0) * nzNormVals.size()));
        if (idx >= nzNormVals.size()) idx = nzNormVals.size() - 1;
        threshold = nzNormVals[idx];
    }
    // Применяем "пол", чтобы серый точно был виден при плоском фоне
    if (threshold < kMinThreshold) threshold = kMinThreshold;

    // --- Геометрия пикселей ---
    const float cellW = static_cast<float>(imgSize) / static_cast<float>(cols);
    const float cellH = static_cast<float>(imgSize) / static_cast<float>(rows);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, false);

    const QColor lowColor(220, 220, 220); // серый для «фона»

    // Цветовая шкала (син->жёлт->красн)
    auto valueToColor = [](double normVal) -> QColor {
        if (normVal < 0.0) normVal = 0.0;
        if (normVal > 1.0) normVal = 1.0;
        if (normVal < 0.5) {
            float t = static_cast<float>(normVal * 2.0);
            int r = static_cast<int>(t * 255.0f);
            int g = static_cast<int>(t * 255.0f);
            int b = static_cast<int>(255.0f * (1.0f - t));
            return QColor(r, g, b);
        }
        else {
            float t = static_cast<float>((normVal - 0.5) * 2.0);
            int r = 255;
            int g = static_cast<int>(255.0f * (1.0f - t));
            int b = 0;
            return QColor(r, g, b);
        }
        };

    if (cellW >= 1.0f && cellH >= 1.0f) {
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                double val = matrix[y][x];
                double normVal = (val - minVal) / (maxVal - minVal);
                if (normVal < 0.0) normVal = 0.0;
                if (normVal > 1.0) normVal = 1.0;

                const QRectF rect(x * cellW, y * cellH, cellW, cellH);
                const QColor color = (normVal <= threshold) ? lowColor : valueToColor(normVal);
                painter.fillRect(rect, color);
            }
        }

        if (cellW >= 2.0f && cellH >= 2.0f) {
            QPen pen(Qt::black);
            pen.setWidth(1);
            painter.setPen(pen);
            for (int y = 0; y <= rows; ++y)
                painter.drawLine(0, y * cellH, imgSize, y * cellH);
            for (int x = 0; x <= cols; ++x)
                painter.drawLine(x * cellW, 0, x * cellW, imgSize);
        }
    }
    else {
        // Фоллбек: nearest-neighbor по пикселям
        for (int py = 0; py < imgSize; ++py) {
            int my = static_cast<int>((static_cast<double>(py) / imgSize) * rows);
            if (my < 0) my = 0;
            if (my >= rows) my = rows - 1;
            for (int px = 0; px < imgSize; ++px) {
                int mx = static_cast<int>((static_cast<double>(px) / imgSize) * cols);
                if (mx < 0) mx = 0;
                if (mx >= cols) mx = cols - 1;
                double val = matrix[my][mx];
                double normVal = (val - minVal) / (maxVal - minVal);
                if (normVal < 0.0) normVal = 0.0;
                if (normVal > 1.0) normVal = 1.0;

                const QColor color = (normVal <= threshold) ? lowColor : valueToColor(normVal);
                image.setPixelColor(px, py, color);
            }
        }
    }

    painter.end();
    return image;
}



void HeatMapRenderer::drawSensorsOnMatrix(QImage& image, const std::vector<Sensor>& sensors, int matrixWidth, int matrixHeight, float /*fieldWidth*/, float /*fieldHeight*/) {
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true); // сглаживание для маркеров

    int pixelWidth = image.width();
    int pixelHeight = image.height();

    float cellWidth = pixelWidth / static_cast<float>(matrixWidth);
    float cellHeight = pixelHeight / static_cast<float>(matrixHeight);

    QPen pen(Qt::black);
    pen.setWidth(2);
    painter.setPen(pen);
    painter.setBrush(Qt::white); // Белая заливка круга

    for (const Sensor& sensor : sensors) {
        double matrixX = sensor.get_x();
        double matrixY = sensor.get_y();

        // Центр ячейки в пикселях
        float cx = static_cast<float>(matrixX * cellWidth + cellWidth / 2.0);
        float cy = static_cast<float>(matrixY * cellHeight + cellHeight / 2.0);

        // Радиус круга — вписанный в ячейку (минимум из ширины и высоты / 2)
        float radius = std::min(cellWidth, cellHeight) / 2.0f;
        if (radius < 1.5f) radius = 1.5f;

        painter.drawEllipse(QPointF(cx, cy), radius, radius);
    }

    painter.end();
}

QImage HeatMapRenderer::renderCompleteHeatmap(const std::vector<std::vector<double>>& matrix,
    const std::vector<Sensor>& sensors,
    const std::vector<QPointF>& trailCells)
{
    // Базовый размер heatmap-канвы (потом мы можем масштабировать)
    const int heatmapSize = 1000;
    const int legendWidth = 80;
    const int footerHeight = 10;

    const int fullWidth = heatmapSize + legendWidth + 20;
    const int fullHeight = heatmapSize + footerHeight + 20;

    QImage finalImage(fullWidth, fullHeight, QImage::Format_ARGB32_Premultiplied);
    finalImage.fill(Qt::white);

    QPainter painter(&finalImage);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // --- Тепловая карта ---
    QImage heatmapImage = matrixToHeatmapImage(matrix);

    // Если heatmapImage не того размера, масштабируем по Nearest (чтобы сохранить "пиксельность")
    if (heatmapImage.width() != heatmapSize || heatmapImage.height() != heatmapSize) {
        heatmapImage = heatmapImage.scaled(heatmapSize, heatmapSize, Qt::IgnoreAspectRatio, Qt::FastTransformation);
    }

    painter.drawImage(0, 0, heatmapImage);

    // Нарисовать датчики поверх
    int matrixWidth = matrix[0].size();
    int matrixHeight = matrix.size();
    drawSensorsOnMatrix(heatmapImage, sensors, matrixWidth, matrixHeight, 1.0f, 1.0f);
    painter.drawImage(0, 0, heatmapImage);

    // --- Рисование хвоста (trail) поверх карты ---
    if (!trailCells.empty()) {
        // вычислим сколько пикселей на одну ячейку
        double pixelsPerCellX = static_cast<double>(heatmapImage.width()) / static_cast<double>(matrixWidth);
        double pixelsPerCellY = static_cast<double>(heatmapImage.height()) / static_cast<double>(matrixHeight);

        QPen tailPen(QColor(10, 120, 200, 220));
        double strokeWidth = std::max(1.0, (pixelsPerCellX + pixelsPerCellY) * 0.18);
        tailPen.setWidthF(strokeWidth);
        tailPen.setCapStyle(Qt::RoundCap);
        tailPen.setJoinStyle(Qt::RoundJoin);
        painter.setPen(tailPen);
        painter.setBrush(Qt::NoBrush);

        QPolygonF poly;
        poly.reserve(trailCells.size());
        for (const auto& pt : trailCells) {
            double fx = pt.x();
            double fy = pt.y();
            double px = fx * pixelsPerCellX + pixelsPerCellX * 0.5;
            double py = fy * pixelsPerCellY + pixelsPerCellY * 0.5;
            poly << QPointF(px, py);
        }

        if (!poly.isEmpty()) {
            painter.drawPolyline(poly);

            // кружки на последних k точках
            int k = std::min(static_cast<int>(trailCells.size()), 6);
            for (int i = 0; i < k; ++i) {
                const QPointF& cellPt = trailCells[trailCells.size() - 1 - i];
                double fx = cellPt.x();
                double fy = cellPt.y();
                double px = fx * pixelsPerCellX + pixelsPerCellX * 0.5;
                double py = fy * pixelsPerCellY + pixelsPerCellY * 0.5;
                double r = std::max(2.0, std::min(10.0, strokeWidth * (1.0 + (k - i) * 0.45)));
                QColor outline(10, 120, 200, 250);
                QColor fill(255, 255, 255, 220);
                painter.setBrush(fill);
                painter.setPen(QPen(outline, 1));
                painter.drawEllipse(QPointF(px, py), r, r);
            }

            // яркий маркер для последней точки
            QPointF last = poly.last();
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(255, 80, 80, 230));
            double lastR = std::max(4.0, strokeWidth * 1.3);
            painter.drawEllipse(last, lastR, lastR);
        }
    }

    // --- Легенда справа ---
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
