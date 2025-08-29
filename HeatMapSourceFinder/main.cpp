//main.cpp
#include "HeatMapSourceFinder.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    HeatMapSourceFinder window;
    window.show();
    return app.exec();
}
