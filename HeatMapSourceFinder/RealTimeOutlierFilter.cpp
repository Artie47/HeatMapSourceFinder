// RealTimeOutlierFilter.cpp
#include "RealTimeOutlierFilter.h"
#include <cmath>

const double M_PI = 3.14159265358979323846;


double RealTimeOutlierFilter::haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    auto to_radians = [](double degree) { return degree * M_PI / 180.0; };
    lat1 = to_radians(lat1);
    lon1 = to_radians(lon1);
    lat2 = to_radians(lat2);
    lon2 = to_radians(lon2);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = sin(dlat / 2) * sin(dlat / 2) +
        cos(lat1) * cos(lat2) *
        sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return 6371.0 * c; // Earth radius in km
}

bool RealTimeOutlierFilter::is_outlier(double lat, double lon, int distance) {
    // Если много выбросов подряд - ослабляем критерии
    if (consecutive_outliers_ >= max_consecutive_outliers_) {
        adaptive_distance_threshold_ *= 1.5;
        adaptive_coord_threshold_km_ *= 1.5;
        consecutive_outliers_ = 0;

    }

    // Стандартная проверка (как раньше), но с адаптивными порогами
    if (recent_distances_.size() < 5) {
        return false;
    }

    double mean_dist = std::accumulate(recent_distances_.begin(),
        recent_distances_.end(), 0.0) / recent_distances_.size();
    double sq_sum = std::inner_product(recent_distances_.begin(), recent_distances_.end(),
        recent_distances_.begin(), 0.0);
    double std_dist = std::sqrt(sq_sum / recent_distances_.size() - mean_dist * mean_dist);

    if (std_dist > 0 && std::abs(distance - mean_dist) > adaptive_distance_threshold_ * std_dist) {
        consecutive_outliers_++;
        return true;
    }

    if (!recent_coords_.empty()) {
        auto [last_lat, last_lon] = recent_coords_.back();
        double distance_km = haversine_distance(last_lat, last_lon, lat, lon);
        if (distance_km > adaptive_coord_threshold_km_) {
            consecutive_outliers_++;
            return true;
        }
    }

    // Сбрасываем счетчик выбросов при успешном измерении
    consecutive_outliers_ = 0;

    // Постепенно возвращаем пороги к исходным значениям
    adaptive_distance_threshold_ = std::max(distance_threshold_,
        adaptive_distance_threshold_ * 0.95);
    adaptive_coord_threshold_km_ = std::max(coord_threshold_km_,
        adaptive_coord_threshold_km_ * 0.95);

    return false;
}


void RealTimeOutlierFilter::add_measurement(double lat, double lon, int distance) {
    recent_distances_.push_back(distance);
    recent_coords_.emplace_back(lat, lon);

    // Maintain window size
    if (recent_distances_.size() > window_size_) {
        recent_distances_.pop_front();
    }
    if (recent_coords_.size() > window_size_) {
        recent_coords_.pop_front();
    }
}

void RealTimeOutlierFilter::reset() {
    recent_distances_.clear();
    recent_coords_.clear();
}

void RealTimeOutlierFilter::set_parameters(size_t window_size, double distance_threshold, double coord_threshold_km) {
    window_size_ = window_size;
    distance_threshold_ = distance_threshold;
    coord_threshold_km_ = coord_threshold_km;
    reset();
}