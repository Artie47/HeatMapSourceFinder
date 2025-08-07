#include "Sensor.h"
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <numbers>

using namespace std;

Sensor::Sensor(double x, double y, double std_dev, double dist_from_src) {
	this->x = x;
	this->y = y;
	this->std_dev = std_dev;
	this->dist_from_src = dist_from_src;
}

Sensor::~Sensor() {
}	

double Sensor::get_x() const {
	return this->x;
}

double Sensor::get_y() const {
	return this->y;
}

double Sensor::get_std_dev() const {
	return this->std_dev;
}

void Sensor::set_x(double x) {
	this->x = x;
}

void Sensor::set_y(double y) {
	this->y = y;
}

void Sensor::set_std_dev(double std_dev) {
	this->std_dev = std_dev;
}

void Sensor::set_dist(double dist) {
	this->dist_from_src = dist;
}

double Sensor::get_dist() const {
	return this->dist_from_src;
}

double Sensor::propability_at(double x, double y) const {
	return (1 / (this->get_std_dev() * sqrt(2 * M_PI))) * exp(-0.5 * pow((sqrt(pow(x - this->x, 2) + pow(y - this->y, 2)) - this->dist_from_src) / std_dev, 2));
}