//SourceEstimator.h

#pragma once

#include <vector>
#include <utility>
using std::vector;
using std::pair;

#include "Sensor.h"

class SourceEstimator
{
private:
    vector<Sensor> sensors;

public:
    SourceEstimator();
	~SourceEstimator(); 

    vector<vector<double>> computeProbabilityMatrix(
        const vector<Sensor>& sensors,
        int x_steps,
        int y_steps
    );

    static pair<double, double> estimateMostLikelyLocation(
        const vector<vector<double>>& matrix,
        double x_min, double x_max,
        double y_min, double y_max
    );

    /*pair<double, double> indexToWorldCoords(
        int i, int j,
        double x_min, double x_max,
        double y_min, double y_max,
        int x_steps, int y_steps
    );*/

};