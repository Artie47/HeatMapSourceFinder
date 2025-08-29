#pragma once

#include "PlaceCalcI.h"
#include <vector>
#include <mutex>
#include <memory>
#include <string>
#include <ctime>
#include <deque>

class PlaceCalcImpl : public PlaceCalcI
{
public:
    PlaceCalcImpl();
    virtual ~PlaceCalcImpl();

    void __stdcall setListener(PlaceCalcListener* listener) override;
    void __stdcall setFindStep(unsigned int step_m) override;
    void __stdcall setMeasureLifeTime(int seconds) override;
    void __stdcall reset() override;
    void __stdcall addMeasure(double ownLatitude, double ownLongitude, int distance_m, const time_t& dateTime) override;
    void __stdcall calc() override;

    bool loadMeasuresFromFile(const std::string& filename, const std::string& sensorId);

    void setGridSize(unsigned int gridWidth, unsigned int gridHeight);
    long long getLastCalcTimeMs() const { return m_lastCalcTimeMs; }
    size_t getLastUsedMeasures() const { return m_lastUsedMeasures; }
    std::string getLastSensorId() const { return m_lastSensorId; }
    double getLastStddevMeters() const { return m_lastStddevMeters; }

    void addMeasurement(double value);
    double getSigma() const { return sigmaSmoothed; }
    const std::deque<double>& getWindow() const { return window; }


private:
    void updateSigmaWindow(int distance_m);
    PlaceCalcListener* m_listener = nullptr;
    unsigned int m_step = 10;
    int m_measureLifeTime = 60;
    unsigned int m_gridWidth = 30;
    unsigned int m_gridHeight = 30;
    double m_lastStddevMeters = 0.0;

    struct Measure {
        double latitude;
        double longitude;
        int distance;
        time_t time;
    };

    std::vector<Measure> m_measures;
    std::mutex m_mutex;

    void notifyResult(double latitude,
        double longitude,
        const std::vector<float>& probabilityZone,
        double zoneLeftTopLatitude,
        double zoneLeftTopLongitude,
        double zoneRightBottomLatitude,
        double zoneRightBottomLongitude,
        unsigned int zoneWidth,
        unsigned int zoneHeight);

    void notifyError(const char* message);

    long long m_lastCalcTimeMs = 0;

    size_t m_lastUsedMeasures = 0;
    std::string m_lastSensorId;

    std::deque<double> window;   // последние 30 измерений
    double sigmaSmoothed;        // сглаженная σ
    const size_t windowSize = 30;

    // параметры фильтра
    double alpha = 0.2;          // сглаживание σ
    double trimPercent = 0.1;    // 10% обрезка выбросов

    // хелпер: вычисляет robust σ по окну
    double computeRobustSigma(const std::deque<double>& w);
    // хелпер: чистит окно от явных выбросов
    void denoiseWindow(std::deque<double>& w);


};

class PlaceCalcListener
{
public:
    virtual ~PlaceCalcListener() = default; 

    virtual void __stdcall onCalcResult(
        double latitude,
        double longitude,
        float* probabilityZone,
        double zoneLeftTopLatitude,
        double zoneLeftTopLongitude,
        double zoneRightBottomLatitude,
        double zoneRightBottomLongitude,
        unsigned int zoneWidth,
        unsigned int zoneHeight
    ) = 0;
    virtual void __stdcall onError(const char* message) = 0;
    // Новый метод — расширенный результат
};