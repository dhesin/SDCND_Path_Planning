//
//  Header.h
//  Path-Planning-XCode
//
//  Created by Esin Darici on 8/11/17.
//  Copyright © 2017 Esin Darici. All rights reserved.
//

#ifndef Header_h
#define Header_h

#include <stdio.h>
#include <iostream>
#include <vector>
#include "util.h"

using namespace std;

struct s_prediction {
  int vehicleId;
  int prediction;
};


struct s_other_vehicle {
  int ind;
  float x;
  float y;
  float vx;
  float vy;
  float s;
  float d;
  float vs;
  float vd;
  float v;
};

enum e_states {
  KEEP_LANE = 0,
  GO_LEFT,
  GO_RIGHT,
  COLD_START
};

using namespace std;


class Vehicle {
  
  // current values of the ego vehicle
  double s;
  double d;
  double x;
  double y;
  double speed;
  double prev_speed;
  double yaw;
  int goal_lane;
  bool lane_change_in_progress;
  e_states state;
  
  double target_speed;
  double avg_acc_with_remaining;
    
  double ref_s;
  double ref_d;
  double ref_x;
  double ref_y;
  double ref_yaw;
  unsigned long ref_ind;

  vector<vector<double>> ego_goals;
  vector<double> ego_predict_costs;
  vector<vector<double>> ego_predictions_x;
  vector<vector<double>> ego_predictions_y;
  vector<vector<double>> ego_predictions_s;
  vector<vector<double>> ego_predictions_d;
  vector<double> ego_predict_speed;
  
  vector<vector<double>> predictions_x;
  vector<vector<double>> predictions_y;
  vector<vector<double>> predictions_s;
  vector<vector<double>> predictions_d;
  vector<s_other_vehicle> other_vehicle_states;
  
  vector<vector<double>> coeefs_s;
  vector<vector<double>> coeffs_d;
  
  
  vector<double> past_and_remaining_v;
  vector<double> next_v_vals;

  std::vector<double> remaining_path_x;
  std::vector<double> remaining_path_y;
  double prev_list_end[3];
  
  unsigned long num_waypoints;
  std::vector<double> mwp_x;
  std::vector<double> mwp_y;
  std::vector<double> mwp_s;
  std::vector<double> mwp_dx;
  std::vector<double> mwp_dy;
  
public:
  Vehicle(float max_acc, float target_speed, int num_lanes,
          vector<double> wp_x, vector<double> wp_y,
          vector<double> wp_s, vector<double> wp_dx, vector<double> wp_dy);
  
  void set_state(double s, double d, double x, double y, double v, double yaw);
  void clear_other_vehicles();
  void set_other_vehicle(int ind, float x, float y, float vx, float vy, float s, float d);
  void set_other_vehicle_2(int ind, float x, float y, float vx, float vy, float s, float d);

  void clear_remaining_path();
  void set_remaining_path(double rp_x, double rp_y);
  void set_prev_list_end(double x, double y);
  void get_trajectory(vector<double>& next_x, vector<double>& next_y, int goal_lane);
  vector<double> JMT(vector<double> start, vector<double>end, double T);
  void get_ego_goals(double goal_s, double goal_d, double goal_t, double ref_s);
  void get_ego_predictions(vector<double> pts_x, vector<double> pts_y);
  void get_ego_predictions_2(vector<double> pts_x, vector<double> pts_y);
  vector<double> check_collision(int i);
  int drive_plan_slow_down();
  int drive_plan_speed_up();
  int drive_plan_constant_speed();
  void get_cost(double goal_s, double goal_d);
  bool check_speed_violation(int i);
  void check_lanes();
  void check_lanes_2();

  void drive_plan(vector<double>& next_x_vals, vector<double>& next_y_vals);
  double get_averages_with_remaining_path();
  
};

#endif /* Header_h */
