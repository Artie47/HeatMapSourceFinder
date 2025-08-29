// LogProcessor.h
#pragma once

#include <string>
#include <vector>
#include <utility>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <streambuf>
#include "RealTimeOutlierFilter.h"

class PlaceCalcI;

enum class ReadMode { Offline, Online };

class LogProcessor {
public:
    explicit LogProcessor(PlaceCalcI* calc);
    ~LogProcessor();

    void processFile(const std::string& path, ReadMode mode, int intervalSeconds);
    bool loadFromFile(const std::string& filename, const std::string& sensorId);
    void stop();

    bool parseLine(const std::string& line);
    std::vector<std::pair<double, double>> getCarriers();
    std::vector<std::pair<double, double>> getLastCarrierPositions();

    // Методы для настройки фильтра
    void set_filter_parameters(size_t window_size, double distance_threshold, double coord_threshold_km);

private:
    void runOffline(const std::string& path);
    void runOnline(const std::string& path, int intervalSeconds);

private:
    PlaceCalcI* m_calc = nullptr;
    RealTimeOutlierFilter outlier_filter_;

    std::thread m_thread;
    std::mutex  m_mutex;
    bool        m_stop = false;
    std::streampos m_lastFilePos{};

    std::vector<std::pair<double, double>> m_carriers;
    std::unordered_map<std::string, std::pair<double, double>> m_lastPositions;

    static constexpr const char* kSingleSensorId = "_single";
};