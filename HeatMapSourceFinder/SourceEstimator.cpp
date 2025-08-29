//SourceEstimator.cpp

#include "SourceEstimator.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
using namespace std;


SourceEstimator::SourceEstimator() {}

SourceEstimator::~SourceEstimator() {
}

vector<vector<double>> SourceEstimator::computeProbabilityMatrix(
    const vector<Sensor>& sensors,
    int x_steps,
    int y_steps
) {
    vector<vector<double>> log_probs(y_steps, vector<double>(x_steps, 0.0));

    double max_logp = -numeric_limits<double>::infinity();

    for (int j = 0; j < y_steps; ++j) {
        for (int i = 0; i < x_steps; ++i) {
            double x = i;
            double y = j;

            double logp = 0.0;
            for (const auto& sensor : sensors) {
                double d = sqrt(pow(x - sensor.get_x(), 2) + pow(y - sensor.get_y(), 2));
                double diff = d - sensor.get_dist();
                double sigma2 = pow(sensor.get_std_dev(), 2);
                logp += -(diff * diff) / (2.0 * sigma2);
            }

            log_probs[j][i] = logp;
            if (logp > max_logp) max_logp = logp;
        }
    }

    for (int j = 0; j < y_steps; ++j)
        for (int i = 0; i < x_steps; ++i)
            log_probs[j][i] = exp(log_probs[j][i] - max_logp);

    return log_probs;
}

pair<double, double> SourceEstimator::estimateMostLikelyLocation(
    const vector<vector<double>>& matrix,
    double x_min, double x_max,
    double y_min, double y_max
) {
    int max_i = 0, max_j = 0;
    double max_val = matrix[0][0];

    for (int j = 0; j < matrix.size(); ++j) {
        for (int i = 0; i < matrix[j].size(); ++i) {
            if (matrix[j][i] > max_val) {
                max_val = matrix[j][i];
                max_i = i;
                max_j = j;
            }
        }
    }
    return { max_i, max_j };
}

/*pair<double, double> SourceEstimator::indexToWorldCoords(
    int i, int j,
    double x_min, double x_max,
    double y_min, double y_max,
    int x_steps, int y_steps
) {
    double dx = (x_max - x_min) / (x_steps - 1);
    double dy = (y_max - y_min) / (y_steps - 1);

    double x = x_min + i * dx;
    double y = y_min + j * dy;

    return { x, y };
}*/