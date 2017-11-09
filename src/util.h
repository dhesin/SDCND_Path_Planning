//
//  util.hpp
//  Path-Planning-XCode
//
//  Created by Esin Darici on 8/23/17.
//  Copyright © 2017 Esin Darici. All rights reserved.
//

#ifndef util_hpp
#define util_hpp

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#define MAX_S  30
#define MAX_D  12

#define Q_DELTA_T 0.02

#define STATE_SIZE 4

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
double logistic(double x);

#endif /* util_hpp */
