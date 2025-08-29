//Sensor.h
#pragma once

class Sensor
{
private:
	double x, y, std_dev, dist_from_src;
public:

	Sensor();
	Sensor(double x, double y, double std_dev, double dist_from_src);

	~Sensor();
	double get_x() const;
	double get_y() const;
	double get_std_dev() const;
	double get_dist() const;
	void set_x(double x);
	void set_y(double y);
	void set_std_dev(double std_dev);
	void set_dist(double dist);

	double propability_at(double x, double y) const;
};