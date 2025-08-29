// RealTimeOutlierFilter.h
#pragma once

#include <deque>
#include <utility>
#include <cmath>
#include <algorithm>
#include <numeric>

class RealTimeOutlierFilter {
private:
    std::deque<double> recent_distances_;
    std::deque<std::pair<double, double>> recent_coords_;
    size_t window_size_;
    double distance_threshold_;
    double coord_threshold_km_;


public:
    static double haversine_distance(double lat1, double lon1, double lat2, double lon2);

    RealTimeOutlierFilter(size_t window_size = 30,
        double distance_threshold = 3.0,
        double coord_threshold_km = 1.0)
        : window_size_(window_size)
        , distance_threshold_(distance_threshold)
        , coord_threshold_km_(coord_threshold_km)
        , adaptive_distance_threshold_(distance_threshold)
        , adaptive_coord_threshold_km_(coord_threshold_km)
        , consecutive_outliers_(0)
    {
    }
    bool is_outlier(double lat, double lon, int distance);
    void add_measurement(double lat, double lon, int distance);
    void reset();

    // Методы для настройки параметров
    void set_parameters(size_t window_size, double distance_threshold, double coord_threshold_km);
    size_t get_window_size() const { return window_size_; }
    double get_distance_threshold() const { return distance_threshold_; }
    double get_coord_threshold_km() const { return coord_threshold_km_; }

    double adaptive_distance_threshold_;
    double adaptive_coord_threshold_km_;
    int consecutive_outliers_;
    const int max_consecutive_outliers_ = 5;
};

