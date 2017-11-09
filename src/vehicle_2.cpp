//
//  vehicle_2.cpp
//  Path-Plan-2
//
//  Created by Esin Darici on 9/28/17.
//  Copyright © 2017 Esin Darici. All rights reserved.
//

#include <stdio.h>
#include <random>

#include "vehicle.h"


void Vehicle::check_lanes()
{
  double s1, d1, v1, l1;
  int current_lane = -1;
  int left_lane = -1;
  int right_lane = -1;
  double cf_l_space=10e10, cf_l_v_diff=10e10, lf_l_space=10e10, lf_l_v_diff=10e10, rf_l_space=10e10, rf_l_v_diff=10e10;
  double cb_l_space=10e10, cb_l_v_diff=10e10, lb_l_space=10e10, lb_l_v_diff=10e10, rb_l_space=10e10, rb_l_v_diff=10e10;
  vector<double> goal;

  current_lane = (int) d/4.0;
  assert(current_lane >=0 && current_lane <=2);
  left_lane = current_lane-1;
  right_lane = current_lane+1;
  
  //cout << "ego s/d/v/lane:" << s << "/" << d << "/" << speed << "/" << current_lane << endl;
  
  for (int i=0; i<other_vehicle_states.size(); i++) {
    s1 = other_vehicle_states[i].s;
    d1 = other_vehicle_states[i].d;
    v1 = other_vehicle_states[i].v;
    l1 = (int)(d1/4.0);
    assert(l1 >=0 && l1 <= 2);
    
    //cout << "vehicle s/d/v/lane :" << s1 << "/" << d1 << "/" << v1 << "/" << l1 << endl;
    
    if (l1==current_lane && s1>=s) {
      if (s1-s < cf_l_space) {
        cf_l_space = s1-s;
        cf_l_v_diff = v1-speed;
      }
    }
    else if (l1==left_lane && s1>=s) {
      if (s1-s < lf_l_space) {
        lf_l_space = s1-s;
        lf_l_v_diff = v1-speed;
      }
    }
    else if (l1==right_lane && s1>=s) {
      if (s1-s < rf_l_space) {
        rf_l_space = s1-s;
        rf_l_v_diff = v1-speed;
      }
    }
    else if (l1==current_lane && s1<s) {
      if (s-s1 < cb_l_space) {
        cb_l_space = s-s1;
        cb_l_v_diff = speed-v1;
      }
    }
    else if (l1==left_lane && s1<s) {
      if (s-s1 < lb_l_space) {
        lb_l_space = s-s1;
        lb_l_v_diff = speed-v1;
      }
    }
    else if (l1==right_lane && s1<s) {
      if (s-s1 < rb_l_space) {
        rb_l_space = s-s1;
        rb_l_v_diff = speed-v1;
      }
    }
  }
  
  //cout << "forward spaces :" << lf_l_space << "/" << cf_l_space << "/" << rf_l_space << endl;
  //cout << "back spaces    :" << lb_l_space << "/" << cb_l_space << "/" << rb_l_space << endl;
  

  double max_speed = 22.5;
  double max_accelaration = 0.1;
  double max_decelaratioin = 0.2;
  double desired_time_fwd = 2.0;
  double desired_time_bwd = 2.0;
  double delta_t;
  double fwd_desired_space_cl = max_speed*desired_time_fwd;
  double fwd_desired_space_rl = max_speed*desired_time_fwd;
  double fwd_desired_space_ll = max_speed*desired_time_fwd;
  double bwd_desired_space_cl = max_speed*desired_time_bwd;
  double bwd_desired_space_rl = max_speed*desired_time_bwd;
  double bwd_desired_space_ll = max_speed*desired_time_bwd;
  
  //double min_fwd_space_cl = desired_time*target_speed;
  
  if (cf_l_v_diff < 0) {
    delta_t = -cf_l_v_diff/max_accelaration;
    fwd_desired_space_cl += 0.5*(-cf_l_v_diff*delta_t*delta_t);
    //cout << "min space changed to :" << min_space;
  }
  if (rf_l_v_diff < 0) {
    delta_t = -rf_l_v_diff/max_accelaration;
    fwd_desired_space_rl += 0.5*(-rf_l_v_diff*delta_t*delta_t);
    //cout << "min space changed to :" << min_space;
  }
  if (lf_l_v_diff < 0) {
    delta_t = -lf_l_v_diff/max_accelaration;
    fwd_desired_space_ll += 0.5*(-lf_l_v_diff*delta_t*delta_t);
    //cout << "min space changed to :" << min_space;
  }

  
  if (cb_l_v_diff < 0) {
    delta_t = -cb_l_v_diff/max_accelaration;
    bwd_desired_space_cl += 0.5*(-cb_l_v_diff*delta_t*delta_t);
    //cout << "min space changed to :" << min_space;
  }
  if (rb_l_v_diff < 0) {
    delta_t = -rb_l_v_diff/max_accelaration;
    bwd_desired_space_rl += 0.5*(-rb_l_v_diff*delta_t*delta_t);
    //cout << "min space changed to :" << min_space;
  }
  if (lb_l_v_diff < 0) {
    delta_t = -lb_l_v_diff/max_accelaration;
    bwd_desired_space_ll += 0.5*(-lb_l_v_diff*delta_t*delta_t);
    //cout << "min space changed to :" << min_space;
  }
 
  
  ref_ind = this->remaining_path_x.size();
  
  if (state == COLD_START or cf_l_space >= fwd_desired_space_cl) {
    
    if (state != COLD_START) {
      state = KEEP_LANE;
      max_accelaration = 0.1;
    }
    else {
      max_accelaration = 0.2;
    }
    
    prev_speed = target_speed;
    double acc_with_remain = get_averages_with_remaining_path();
    if (target_speed < max_speed && acc_with_remain < max_accelaration) {
      target_speed = target_speed + (max_accelaration-acc_with_remain);
      if (target_speed > max_speed) {
        target_speed = max_speed;
      }
      //cout << "C1 new target speed :" << target_speed << endl;
    }
    
    goal_lane = current_lane;
    //cout << "CASE 1 KEEP --->  acc :" << target_speed-prev_speed << endl;
  }
  else if (left_lane >= 0 && lf_l_space > fwd_desired_space_ll && lb_l_space >= bwd_desired_space_ll) {
    state = GO_LEFT;
    
    if (remaining_path_x.size() >= 5)
      ref_ind = 5;
    lane_change_in_progress = true;
    
    prev_speed = target_speed;
    double acc_with_remain = get_averages_with_remaining_path();
    if (target_speed < max_speed && acc_with_remain < max_accelaration) {
      target_speed = target_speed + (max_accelaration-acc_with_remain);
      if (target_speed > max_speed) {
        target_speed = max_speed;
      }
      //cout << "C3 new target speed :" << target_speed << endl;
    }
    goal_lane = left_lane;
    //cout << "CASE 3 LEFT ---> acc :" << target_speed-prev_speed << endl;
  }
  else if (right_lane <= 2 && rf_l_space > fwd_desired_space_rl && rb_l_space >= bwd_desired_space_rl) {
    state = GO_RIGHT;
    
    if (remaining_path_x.size() >= 5)
      ref_ind = 5;
    lane_change_in_progress = true;
    
    prev_speed = target_speed;
    double acc_with_remain = get_averages_with_remaining_path();
    if (target_speed < max_speed && acc_with_remain < max_accelaration) {
      target_speed = target_speed + (max_accelaration-acc_with_remain);
      if (target_speed > max_speed) {
        target_speed = max_speed;
      }
      //cout << "C4 new target speed :" << target_speed << endl;
    }
    
    goal_lane = right_lane;
    //cout << "CASE 4 RIGHT--->  acc :" << target_speed-prev_speed << endl;
  }
  else {
    state = KEEP_LANE;
    
    if (remaining_path_x.size() >= 5)
      ref_ind = 5;
    
    prev_speed = target_speed;
    double acc = 0.0;
    double acc_with_remain = get_averages_with_remaining_path();
    assert(acc_with_remain >= 0);
    if (cf_l_space < 30) {
      max_decelaratioin = 1.0;
    }
    if (acc_with_remain < max_decelaratioin) {
      acc = acc_with_remain-max_decelaratioin;
    }
    target_speed = target_speed + acc;
    goal_lane = current_lane;
    //cout << "CASE 5 KEEP ---> acc/cf_l_v_diff/target :" << target_speed-prev_speed << "/" << cf_l_v_diff << "/" << target_speed << endl;
  }
}


double Vehicle::get_averages_with_remaining_path() {
  
  // add used points from next_v_vals
  for (int i=0; i<50-remaining_path_x.size() && i<next_v_vals.size(); i++) {
    past_and_remaining_v.push_back(next_v_vals[i]);
  }
  
  // add remaining path up to new reference index
  for (unsigned long i=50-remaining_path_x.size(); i<50-remaining_path_x.size()+ref_ind && i<next_v_vals.size(); i++) {
    past_and_remaining_v.push_back(next_v_vals[i]);
  }

  
  //cout << "size after copy :" << past_trajectory_y.size() << endl;
  unsigned long size = past_and_remaining_v.size();
  if (size > 60) {
    past_and_remaining_v.erase(past_and_remaining_v.begin(), past_and_remaining_v.begin()+size-60);
  }
  
  double min_speed = 1e10, max_speed = 0;
  avg_acc_with_remaining = 0.0;
  
  for (int i=0; i<past_and_remaining_v.size(); i++) {
    if (past_and_remaining_v[i] < min_speed)
      min_speed = past_and_remaining_v[i];
    if (past_and_remaining_v[i] > max_speed)
      max_speed = past_and_remaining_v[i];
    
    //if (abs(past_v[i+1]-past_v[i]) > avg_acceleration) {
    //  avg_acceleration = abs(past_v[i+1]-past_v[i]);
    //}
  }
  if (past_and_remaining_v.size() > 0) {
    avg_acc_with_remaining = max_speed-min_speed;
  }
  //cout << "avg_acc_with_remaining: " << avg_acc_with_remaining << "/" << past_and_remaining_v.size() << endl;
  return avg_acc_with_remaining;
  //return 0;
}

void Vehicle::drive_plan(vector<double> &next_x_vals, vector<double> &next_y_vals) {
  
  if (state == COLD_START) {
    if (speed > 15) {
      state = KEEP_LANE;
    }
  }
  
  if (lane_change_in_progress) {
    int current_lane = (int) d/4.0;
    int lane_shift = fmod(d, 4.0);
    if (current_lane == goal_lane && lane_shift >= 1.5 && lane_shift <= 2.5)
      lane_change_in_progress = false;
  }
  
  if (!lane_change_in_progress) {
    check_lanes();
  }
  else {
    //cout << "lane changing " << endl;
    state = KEEP_LANE;
  }
  
  
  get_trajectory(next_x_vals, next_y_vals, goal_lane);
  
}
