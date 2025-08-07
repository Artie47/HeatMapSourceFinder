#pragma once

#include <QMainWindow>

namespace Ui {
    class HeatMapSourceFinderClass;
}

class HeatMapSourceFinder : public QMainWindow
{
    Q_OBJECT

public:
    explicit HeatMapSourceFinder(QWidget* parent = nullptr);
    ~HeatMapSourceFinder();

private:
    Ui::HeatMapSourceFinderClass* ui;
};
