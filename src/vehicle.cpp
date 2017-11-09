//
//  vehicle.cpp
//  Path-Planning-XCode
//
//  Created by Esin Darici on 8/11/17.
//  Copyright © 2017 Esin Darici. All rights reserved.
//



#include <stdio.h>
#include <iostream>
#include <random>
#include <limits>
//#include <float.h>

#include "vehicle.h"
#include "util.h"
#include "Eigen-3.3/Eigen/Dense"
#include "MPC.h"
#include "spline.h"


extern int MAX_N;
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
tk::spline spline2;


Vehicle::Vehicle(float max_acc, float target_speed, int num_lanes,
                 vector<double> wp_x, vector<double> wp_y,
                 vector<double> wp_s, vector<double> wp_dx, vector<double> wp_dy) {
    
  this->num_waypoints = wp_x.size();
  this->mwp_x = wp_x;
  this->mwp_y = wp_y;
  this->mwp_dx = wp_dx;
  this->mwp_dy = wp_dy;
  this->mwp_s = wp_s;
  this->target_speed = 0;
  goal_lane = 1;
  lane_change_in_progress = false;
  state = COLD_START;
  avg_acc_with_remaining = 0.0;
  prev_speed = 0.0;
  
}

void Vehicle::set_state(double s, double d, double x, double y, double v, double yaw) {
  
  this->s = s;
  this->d = d;
  this->x = x;
  this->y = y;
  this->speed = v*0.447;
  this->yaw = deg2rad(yaw);
  
  //cout << "current readings s/d/x/y/speed/yaw/target_speed: " << s << "," << d << "," << x << "," << y << "," << speed << "," << yaw << "," << target_speed << endl;
}

void Vehicle::set_prev_list_end(double s, double d) {
  this->prev_list_end[0] = s;
  this->prev_list_end[1] = d;
  //cout << "prev list ends: " << s << "," << d << endl;
}

void Vehicle::clear_other_vehicles()
{
  predictions_x.clear();
  predictions_y.clear();
  predictions_s.clear();
  predictions_d.clear();
  other_vehicle_states.clear();

}
void Vehicle::set_other_vehicle(int ind, float x_o, float y_o, float vx_o, float vy_o, float s_o, float d_o) {
  
  double v;
  vector<double> predict_x, predict_s;
  vector<double> predict_y, predict_d;
  s_other_vehicle other_vehicle;
  
  double car_theata = atan2(vy_o, vx_o);
  v = sqrt(vx_o*vx_o+vy_o*vy_o);
  
  if (v == 0.0)
    return;
  
  if (s_o > s+200)
    return;
  
  if (s_o < s-200)
    return;
    
  int nwp_ind = NextWaypoint(x_o, y_o, car_theata, mwp_x, mwp_y);
  nwp_ind--;
  double wp_dx = mwp_dx[nwp_ind];
  double wp_dy = mwp_dy[nwp_ind];
  double wp_theata = atan2(wp_dy, wp_dx);
  
  double car2sd_theata = wp_theata-car_theata;
  double vs = v*cos(car2sd_theata);
  double vd = v*sin(car2sd_theata);
  
  other_vehicle.x = x_o;
  other_vehicle.y = y_o;
  other_vehicle.vx = vx_o;
  other_vehicle.vy = vy_o;
  other_vehicle.vs = vs;
  other_vehicle.vd = vd;
  other_vehicle.v = v;
  other_vehicle.s = s_o;
  other_vehicle.d = d_o;
  other_vehicle_states.push_back(other_vehicle);
  
//  vd = vx_o*cos(-wp_theata)-vy_o*sin(-wp_theata);
//  vs = vx_o*sin(-wp_theata)+vy_o*cos(-wp_theata);
//  double step_s = vs*Q_DELTA_T;
//  double step_d = vd*Q_DELTA_T;
//  
//  //cout << "others :" << ind << "/" << v << "/" << vs << "/" << vd << "/" << car2sd_theata << endl;
//  
//  double next_s_car;
//  double next_d_car;
//  
//  for (int i=1; i<=50; i++) {
//    
//    next_s_car = s_o+step_s*i;
//    next_d_car = d_o+step_d*i;
//    vector<double> XY = getXY(next_s_car, next_d_car, mwp_s, mwp_x, mwp_y);
//    
//    predict_x.push_back(XY[0]);
//    predict_y.push_back(XY[1]);
//    predict_s.push_back(next_s_car);
//    predict_d.push_back(next_d_car);
//    //cout << sd[0] << "/" << sd[1] << " ";
//    
//  }
//  //cout << endl;
//  
//  predictions_x.push_back(predict_x);
//  predictions_y.push_back(predict_y);
//  
//  predictions_s.push_back(predict_s);
//  predictions_d.push_back(predict_d);
  
}

void Vehicle::clear_remaining_path() {
  this->remaining_path_x.clear();
  this->remaining_path_y.clear();
  assert(remaining_path_x.size() == 0);
}

void Vehicle::set_remaining_path(double rp_x, double rp_y) {
  
  this->remaining_path_x.push_back(rp_x);
  this->remaining_path_y.push_back(rp_y);

}

void Vehicle::get_trajectory(vector<double>& next_x_vals, vector<double>& next_y_vals, int goal_lane) {
  
  vector<double> ptsx;
  vector<double> ptsy;
  double ref_x = this->x;
  double ref_y = this->y;
  double ref_vel = this->target_speed;
  double ref_yaw = this->yaw;
  double prev_x, prev_y;

  next_v_vals.clear();
  
  if (ref_ind < 2) {
    prev_x = x - cos(yaw);
    prev_y = y - sin(yaw);
    
    ptsx.push_back(prev_x);
    ptsx.push_back(x);
    ptsy.push_back(prev_y);
    ptsy.push_back(y);
  }
  else {
    ref_x = remaining_path_x[ref_ind-1];
    ref_y = remaining_path_y[ref_ind-1];
    prev_x = remaining_path_x[ref_ind-2];
    prev_y = remaining_path_y[ref_ind-2];
    ref_yaw = atan2(ref_y-prev_y, ref_x-prev_x);
    //cout << "calculated yaw: " << ref_yaw << " given yaw: " << yaw << endl;
    
    ptsx.push_back(prev_x);
    ptsx.push_back(ref_x);
    ptsy.push_back(prev_y);
    ptsy.push_back(ref_y);
  }
  
  //cout << "prev_x/y, ref_x/y, x/y :" << prev_x << " " << prev_y << " " << ref_x << " " << ref_y << " " << x << " " << y << endl;
 
  double s_temp = s;
  if (s > 6914.14925765991) {
    s = s-6914.14925765991;
  }
  vector<double> wp;
  wp = getXY(s+30,(2+4*goal_lane), mwp_s, mwp_x, mwp_y);
  ptsx.push_back(wp[0]);
  ptsy.push_back(wp[1]);
  
  wp = getXY(s+60,(2+4*goal_lane), mwp_s, mwp_x, mwp_y);
  ptsx.push_back(wp[0]);
  ptsy.push_back(wp[1]);
  
  wp = getXY(s+90,(2+4*goal_lane), mwp_s, mwp_x, mwp_y);
  ptsx.push_back(wp[0]);
  ptsy.push_back(wp[1]);
  
  for (int i=0; i<ptsx.size(); i++) {
    prev_x = ptsx[i]-ref_x;
    prev_y = ptsy[i]-ref_y;
    ptsx[i] = prev_x*cos(-ref_yaw)-prev_y*sin(-ref_yaw);
    ptsy[i] = prev_x*sin(-ref_yaw)+prev_y*cos(-ref_yaw);
    //cout << "pushing for spline x/y " << ptsx[i] << " " << ptsy[i] << " " << endl;
  }
  
  tk::spline spline;
  spline.set_points(ptsx, ptsy);
  
  
  for (int i=0; i<ref_ind; i++) {
    next_x_vals.push_back(remaining_path_x[i]);
    next_y_vals.push_back(remaining_path_y[i]);
    next_v_vals.push_back(prev_speed);
  }
  
  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_distance = sqrt(target_x*target_x+target_y*target_y);
  double N = (target_distance/(Q_DELTA_T*ref_vel))+1.0;
  double step = target_x/N;
  double next_x, next_y, x, y;
  double next_x_car = 0.0, next_y_car;
  
  //cout << "target x/y,N,step :" << target_x << " " << target_y << " " << N << " " << step << endl;
  
  for (int i=1; i<=50-ref_ind; i++) {
    next_x_car = next_x_car+step;
    next_y_car = spline(next_x_car);
    
    x = next_x_car;
    y = next_y_car;
    
    next_x = x*cos(ref_yaw)-y*sin(ref_yaw);
    next_y = x*sin(ref_yaw)+y*cos(ref_yaw);
    next_x += ref_x;
    next_y += ref_y;
    
    next_x_vals.push_back(next_x);
    next_y_vals.push_back(next_y);
    next_v_vals.push_back(target_speed);
  }
  
  s = s_temp;
}
vector<double> Vehicle::JMT(vector< double> start, vector <double> end, double T)
{
  /*
   Calculate the Jerk Minimizing Trajectory that connects the initial state
   to the final state in time T.
   
   INPUTS
   
   start - the vehicles start location given as a length three array
   corresponding to initial values of [s, s_dot, s_double_dot]
   
   end   - the desired end state for vehicle. Like "start" this is a
   length three array.
   
   T     - The duration, in seconds, over which this maneuver should occur.
   
   OUTPUT
   an array of length 6, each value corresponding to a coefficent in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   
   EXAMPLE
   
   > JMT( [0, 10, 0], [10, 10, 0], 1)
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
  3*T*T, 4*T*T*T,5*T*T*T*T,
  6*T, 12*T*T, 20*T*T*T;
		
  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
  end[1]-(start[1]+start[2]*T),
  end[2]-start[2];
  
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }
  
  return result;
  
}
std::random_device rd1, rd2;
default_random_engine generator1, generator2;
normal_distribution<double> dist_d1(2, 1.0);
normal_distribution<double> dist_d2(6, 1.0);
normal_distribution<double> dist_d3(10, 1.0);

vector<normal_distribution<double>> dist_D;


void Vehicle::get_ego_goals(double goal_s, double goal_d, double goal_t, double ref_s) {
 
  // clear previous goals
  ego_goals.clear();
  
  dist_D.push_back(dist_d1);
  dist_D.push_back(dist_d2);
  dist_D.push_back(dist_d3);
  
  normal_distribution<double> dist_s(goal_s, target_speed);

  
  double desired_speed = (goal_s-ref_s)/goal_t;

  for (int dind = -1; dind<=1; dind++) {
    
    double d_try = goal_d+dind*4;
    if (d_try < 0 || d_try > 12)
      continue;
    
    //normal_distribution<double> dist_d(d_try, 1.0);
    
    int num_goals = 0;
    for (int tind=-4; tind<=4; tind++) {
      
      double delta_t = goal_t+tind*0.5;
      if (delta_t <= 0.0)
        continue;
      
      for (int i=0; i<10; i++) {
        
        vector<double> goal;
        double sn = dist_s(generator1);
        while (sn <= (ref_s+1))
          sn = dist_s(generator1);
        
        double dn = dist_D[dind+1](generator2);
        
        //double dn = dist_d(generator2);
        //double lane_offset = fmod(dn,4.0);
        //cout << "d_goal/lane offset/initial s/d ::" << dn << "/" << lane_offset << " " << goal_s << "/" << goal_d << endl;
        //while (dn <= d_try-0.5 || dn >= d_try+0.5 || lane_offset <= 1.8 || lane_offset >= 2.2) {
        while (dn <= d_try-1.5 || dn >= d_try+1.5) {
          dn = dist_D[dind+1](generator2);
          //dn = dist_d(generator2);
          //lane_offset = fmod(dn,4.0);
        }
        
        goal.push_back(sn);
        goal.push_back(dn);
        goal.push_back(delta_t);
        ego_goals.push_back(goal);
        num_goals++;
        
      }
    }
    
  }
  

}

void Vehicle::get_ego_predictions(vector<double> inp_x, vector<double> inp_y) {

  double s_goal, d_goal, t_goal, prev_x, prev_y, diff_s, delta_s, delta_d;
  
  ego_predictions_x.clear();
  ego_predictions_y.clear();
  ego_predict_speed.clear();
  ego_predictions_s.clear();
  ego_predictions_d.clear();
  
  //cout << " ego_goals.size()-->: " << ego_goals.size() << endl;
  
  for (int i=0; i<ego_goals.size(); i++) {
    
    s_goal = ego_goals[i][0];
    d_goal = ego_goals[i][1];
    t_goal = ego_goals[i][2];
    
    //cout << "random goals :" << s_goal << "/" << d_goal << " " << t_goal << endl;

    vector<double> ptsx, ptsy;

    for (int j=0; j<inp_x.size(); j++) {
      ptsx.push_back(inp_x[j]);
      ptsy.push_back(inp_y[j]);
    }
    //cout << "prev_x/y, ref_x/y, x/y :" << prev_x << " " << prev_y << " " << ref_x << " " << ref_y << " " << x << " " << y << endl;
    
    
    vector<double> wp;
    wp = getXY(s_goal, d_goal, mwp_s, mwp_x, mwp_y);
    double x_goal = wp[0];
    double y_goal = wp[1];
    //ptsx.push_back(wp[0]);
    //ptsy.push_back(wp[1]);
    wp = getXY(s_goal+10, d_goal, mwp_s, mwp_x, mwp_y);
    ptsx.push_back(wp[0]);
    ptsy.push_back(wp[1]);
    wp = getXY(s_goal+15, d_goal, mwp_s, mwp_x, mwp_y);
    ptsx.push_back(wp[0]);
    ptsy.push_back(wp[1]);
    wp = getXY(s_goal+20, d_goal, mwp_s, mwp_x, mwp_y);
    ptsx.push_back(wp[0]);
    ptsy.push_back(wp[1]);
    wp = getXY(s_goal+25, d_goal, mwp_s, mwp_x, mwp_y);
    ptsx.push_back(wp[0]);
    ptsy.push_back(wp[1]);
    
    //cout << "pushing for spline size/goal_s/goal_d : " << ptsx.size() << " " << s_goal << " " << d_goal << " ";
    for (int j=0; j<ptsx.size(); j++) {
      prev_x = ptsx[j]-ref_x;
      prev_y = ptsy[j]-ref_y;
      ptsx[j] = prev_x*cos(-ref_yaw)-prev_y*sin(-ref_yaw);
      ptsy[j] = prev_x*sin(-ref_yaw)+prev_y*cos(-ref_yaw);
      //cout << ptsx[j] << " " ;
    }
    //cout << endl;
    
    // convert x_goal, y_goal to car coordinates
    double x_goal_c, y_goal_c;
    x_goal_c = (x_goal-ref_x)*cos(-ref_yaw)-(y_goal-ref_y)*sin(-ref_yaw);
    y_goal_c = (x_goal-ref_x)*sin(-ref_yaw)+(y_goal-ref_y)*cos(-ref_yaw);
    //assert(x_goal_c > 0);
    
    tk::spline spline;
    spline.set_points(ptsx, ptsy);

    
    int N = (int)(t_goal/Q_DELTA_T)+1;
    double step = x_goal_c/N;
    double next_x, next_y, x, y;
    double next_x_car = 0.0, next_y_car, prev_y_car;
    vector<double> predict_x, predict_x_cc, predict_s;
    vector<double> predict_y, predict_y_cc, predict_d;

    //assert(step > 0);
    //cout << "target x/y,N,step :" << x_goal << " " << y_goal << " " << N << " " << step << endl;
 
    double theata = yaw;
    for (int j=0; j<ref_ind; j++) {
      predict_x.push_back(remaining_path_x[j]);
      predict_y.push_back(remaining_path_y[j]);
      
      vector<double> sd = getFrenet(remaining_path_x[j], remaining_path_y[j], theata, mwp_x, mwp_y);
      predict_s.push_back(sd[0]);
      predict_d.push_back(sd[1]);

      if (j >= 1) {
        theata = atan2(remaining_path_y[j]-remaining_path_y[j-1], remaining_path_x[j]-remaining_path_x[j-1]);
      }
    }
    
    double target_distance = 0;
    double perturbed_speed;

    theata = ref_yaw;
    prev_x = ref_x;
    prev_y = ref_y;
    prev_y_car = 0;
    for (int j=ref_ind; j<N; j++) {
      next_x_car = next_x_car+step;
      next_y_car = spline(next_x_car);
      target_distance += sqrt(step*step+(next_y_car-prev_y_car)*(next_y_car-prev_y_car));
      prev_y_car = next_y_car;
      
      predict_x_cc.push_back(next_x_car);
      predict_y_cc.push_back(next_y_car);
      
      x = next_x_car;
      y = next_y_car;
      
      next_x = x*cos(ref_yaw)-y*sin(ref_yaw);
      next_y = x*sin(ref_yaw)+y*cos(ref_yaw);
      next_x += ref_x;
      next_y += ref_y;
      
      predict_x.push_back(next_x);
      predict_y.push_back(next_y);
      
      vector<double> sd = getFrenet(next_x, next_y, theata, mwp_x, mwp_y);
      
      predict_s.push_back(sd[0]);
      predict_d.push_back(sd[1]);
      

      theata = atan2(next_y-prev_y, next_x-prev_x);
      prev_y = next_y;
      prev_x = next_x;
    }
    
    //perturbed_speed = target_distance/t_goal;
    perturbed_speed = sqrt((s_goal-ref_s)*(s_goal-ref_s)+(d_goal-ref_d)*(d_goal-ref_d))/t_goal;
    //perturbed_speed = (s_goal-ref_s)/t_goal;
    
    
    ego_predictions_x.push_back(predict_x);
    ego_predictions_y.push_back(predict_y);
    
    ego_predictions_s.push_back(predict_s);
    ego_predictions_d.push_back(predict_d);

    ego_predict_speed.push_back(perturbed_speed);
    
  }
}
void Vehicle::get_cost(double goal_s, double goal_d) {
  
  ego_predict_costs.clear();
  double cost = 0.0;
  
  for (int i=0; i<ego_predictions_y.size(); i++) {
    
    if (check_speed_violation(i)) {
      ego_predict_costs.push_back(1e11);
      continue;
    }
    
    vector<double> min_dist_rf = check_collision(i);
    if (min_dist_rf[0] <= 2.0) {
      cost += 1e11;
    }
    
    // closeness cost
    if (min_dist_rf[1] < 10)
      cost += logistic(min_dist_rf[1]-10)*(min_dist_rf[1]-10);

    // add goal_s cost
    cost += logistic((ego_goals[i][0]-goal_s)*(ego_goals[i][0]-goal_s));
    // add goal_d cost
    cost += logistic((ego_goals[i][1]-goal_d)*(ego_goals[i][1]-goal_d));
    
    // add lane shift cost
    //double lane_offset = fmod(ego_goals[i][1],4.0);
    //cost += 1e3*(lane_offset-2.0)*(lane_offset-2.0);
    
    //cost += (1.0-logistic(min_dist_rf[0]));
    //cost += (1.0-logistic(min_dist_rf[1]));
    
    
    // add lane change cost
    int current_lane = (int)(d/4.0);
    int next_lane = (int)(ego_goals[i][1]/4.0);
    cost += logistic((goal_lane-next_lane)*(goal_lane-next_lane));
    
    // add acceleration cost
    //if (abs(avg_speed-ego_predict_speed[i]) > 5.0)
    //  cost += logistic((avg_speed-ego_predict_speed[i])*(avg_speed-ego_predict_speed[i]));

    // add target speed cost
    cost += logistic((target_speed-ego_predict_speed[i])*(target_speed-ego_predict_speed[i]));
    
    // goal change cost
    //cost += logistic((ego_goals[i][1]-prev_list_end[1])*(ego_goals[i][1]-prev_list_end[1]));
    
    // add low speed cost
    //if (ego_predict_speed[i]<40)
    //  cost += 10000;

    //if (abs(ego_goals[i][1]-ref_d) > 3)
    //cout << "current/ref/goal D/speed/cost:" << d << "/" << ref_d << "/" << ego_goals[i][1] << "/" << ego_predict_speed[i] << "/" << cost << endl;

    ego_predict_costs.push_back(cost);
  }
  
}
bool Vehicle::check_speed_violation(int i) {
  
  if (ego_predict_speed[i] > 22.0)
    return true;

  return false;
}
vector<double> Vehicle::check_collision(int i) {
  
  bool collision = false;
  vector<double> ego_predict_s(ego_predictions_s[i]);
  vector<double> ego_predict_d(ego_predictions_d[i]);
  double min_dist_r = 1e10, min_dist_f = 1e10;
  
  assert(ego_predict_s.size() == ego_predict_d.size());
  //assert(ego_predict_d.size() == 50);
  
  for (int j=0; j<predictions_s.size(); j++) {
    
    vector<double> predict_s(predictions_s[j]);
    vector<double> predict_d(predictions_d[j]);
    
    assert(predict_d.size() == predict_s.size());
    //assert(predict_s.size() == 50);
    
    for (int k=0; k<ego_predict_s.size(); k++) {
      double dist_s = (predict_s[k]-ego_predict_s[k]);
      double dist_d = (predict_d[k]-ego_predict_d[k]);
      double dist = sqrt(dist_s*dist_s+dist_d*dist_d);
      if (dist < min_dist_r)
        min_dist_r = dist;
      
      if ( abs(dist_d) < 1.0 && dist_s > 0 && dist_s < min_dist_f)
        min_dist_f = dist_s;
      
      if (dist <= 2.0) {
      //  collision = true;
      //  break;
        //cout << i << " is VERY CLOSE to " << j << " at step " << k << " " << dist_s << " " << dist_d << " ";
        //cout << ego_predict_d[k] << "/" << ego_predict_s[k] << "  " << predict_d[k] << "/" << predict_s[k] << endl;
        return {dist, min_dist_f};
      }
    }
    //if (collision)
      //break;
  }
  //return collision;
  return {min_dist_r, min_dist_f};
  
}


void Vehicle::get_ego_predictions_2(vector<double> inp_x, vector<double> inp_y) {
  
  double s_goal, d_goal, t_goal, prev_x, prev_y, diff_s, delta_s, delta_d;
  
  ego_predictions_x.clear();
  ego_predictions_y.clear();
  ego_predict_speed.clear();
  ego_predictions_s.clear();
  ego_predictions_d.clear();
  
  //cout << " ego_goals.size()-->: " << ego_goals.size() << endl;
  
  for (int i=0; i<ego_goals.size(); i++) {
    
    s_goal = ego_goals[i][0];
    d_goal = ego_goals[i][1];
    t_goal = ego_goals[i][2];
    
    //cout << "random goals :" << s_goal << "/" << d_goal << " " << t_goal << endl;
    
    vector<double> ptsx, ptsy;
    
    for (int j=0; j<inp_x.size(); j++) {
      ptsx.push_back(inp_x[j]);
      ptsy.push_back(inp_y[j]);
    }
    //cout << "prev_x/y, ref_x/y, x/y :" << prev_x << " " << prev_y << " " << ref_x << " " << ref_y << " " << x << " " << y << endl;
    
    assert(s_goal > ref_s);
    int num_spline_points = 1;
    double diff_s = s_goal-ref_s;
    delta_s = diff_s/num_spline_points;
    
    while (delta_s > 10)
      delta_s = diff_s/(num_spline_points++);
    
    delta_s = diff_s/num_spline_points;
    delta_d = (d_goal-ref_d)/num_spline_points;
    
    for (int j=1; j<=num_spline_points; j++) {
      vector<double> XY = getXY(ref_s+delta_s*j, ref_d+delta_d*j, mwp_s, mwp_x, mwp_y);
      ptsx.push_back(XY[0]);
      ptsy.push_back(XY[1]);
      
      if (ref_s+delta_s*j >= s_goal)
        break;
    }
    
    vector<double> XY = getXY(s_goal+10, d_goal, mwp_s, mwp_x, mwp_y);
    ptsx.push_back(XY[0]);
    ptsy.push_back(XY[1]);
    
    //XY = getXY(s_goal+18, d_goal, mwp_s, mwp_x, mwp_y);
    //ptsx.push_back(XY[0]);
    //ptsy.push_back(XY[1]);

    XY = getXY(s_goal+20, d_goal, mwp_s, mwp_x, mwp_y);
    ptsx.push_back(XY[0]);
    ptsy.push_back(XY[1]);
    
    //cout << "pushing for spline size/goal_s/goal_d : " << ptsx.size() << " " << s_goal << " " << d_goal << " ";
    for (int j=0; j<ptsx.size(); j++) {
      prev_x = ptsx[j]-ref_x;
      prev_y = ptsy[j]-ref_y;
      ptsx[j] = prev_x*cos(-ref_yaw)-prev_y*sin(-ref_yaw);
      ptsy[j] = prev_x*sin(-ref_yaw)+prev_y*cos(-ref_yaw);
      //cout << ptsx[j] << " " ;
    }
    //cout << endl;
  
    tk::spline spline;
    spline.set_points(ptsx, ptsy);

    // convert x_goal, y_goal to car coordinates
    vector<double> wp;
    wp = getXY(s_goal, d_goal, mwp_s, mwp_x, mwp_y);
    double x_goal = wp[0];
    double y_goal = wp[1];

    double x_goal_c, y_goal_c;
    x_goal_c = (x_goal-ref_x)*cos(-ref_yaw)-(y_goal-ref_y)*sin(-ref_yaw);
    y_goal_c = (x_goal-ref_x)*sin(-ref_yaw)+(y_goal-ref_y)*cos(-ref_yaw);
    //assert(x_goal_c > 0);
    
    
    
    int N = (int)(t_goal/Q_DELTA_T)+1;
    double step = x_goal_c/N;
    double next_x, next_y, x, y;
    double next_x_car = 0.0, next_y_car, prev_y_car;
    vector<double> predict_x, predict_s;
    vector<double> predict_y, predict_d;
    
    //assert(step > 0);
    //cout << "target x/y,N,step :" << x_goal << " " << y_goal << " " << N << " " << step << endl;
    
    double theata = yaw;
    for (int j=0; j<ref_ind; j++) {
      predict_x.push_back(remaining_path_x[j]);
      predict_y.push_back(remaining_path_y[j]);
      
      vector<double> sd = getFrenet(remaining_path_x[j], remaining_path_y[j], theata, mwp_x, mwp_y);
      predict_s.push_back(sd[0]);
      predict_d.push_back(sd[1]);
      
      if (j >= 1) {
        theata = atan2(remaining_path_y[j]-remaining_path_y[j-1], remaining_path_x[j]-remaining_path_x[j-1]);
      }
    }
    
    double target_distance = 0;
    double perturbed_speed;
    
    theata = ref_yaw;
    prev_x = ref_x;
    prev_y = ref_y;
    prev_y_car = 0;
    for (int j=ref_ind; j<50; j++) {
      next_x_car = next_x_car+step;
      next_y_car = spline(next_x_car);
      target_distance += sqrt(step*step+(next_y_car-prev_y_car)*(next_y_car-prev_y_car));
      prev_y_car = next_y_car;
      
      
      x = next_x_car;
      y = next_y_car;
      
      next_x = x*cos(ref_yaw)-y*sin(ref_yaw);
      next_y = x*sin(ref_yaw)+y*cos(ref_yaw);
      next_x += ref_x;
      next_y += ref_y;
      
      predict_x.push_back(next_x);
      predict_y.push_back(next_y);
      
      vector<double> sd = getFrenet(next_x, next_y, theata, mwp_x, mwp_y);
      
      predict_s.push_back(sd[0]);
      predict_d.push_back(sd[1]);
      
      
      theata = atan2(next_y-prev_y, next_x-prev_x);
      prev_y = next_y;
      prev_x = next_x;
    }
    
    //perturbed_speed = target_distance/t_goal;
    perturbed_speed = sqrt((s_goal-ref_s)*(s_goal-ref_s)+(d_goal-ref_d)*(d_goal-ref_d))/t_goal;
    //perturbed_speed = (s_goal-ref_s)/t_goal;
    
    //if (abs(d_goal-ref_d) > 3)
    //  cout << "current/ref/goal D - speed:" << d << "/" << ref_d << "/" << d_goal << "-" << perturbed_speed << endl;
    
    ego_predictions_x.push_back(predict_x);
    ego_predictions_y.push_back(predict_y);
    
    ego_predictions_s.push_back(predict_s);
    ego_predictions_d.push_back(predict_d);
    
    ego_predict_speed.push_back(perturbed_speed);
    
  }
}