#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;
const double pi = acos(-1);
const int maxint = 100000;
const float maxfloat = 100000;
const double maxdouble = 100000;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

      c.clear();
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      //Do for each point
      for(int i = 0; i<n; ++i){
        for(int j = 0; j<m; ++j){
          float dist = old_points[i].distToPoint2(&trans_points[j]);
          if(dist<min_dist){
            min_dist = dist;
            min_index = j;
            second_min_index = j-1;
          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }


}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob)
{

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point. 
  //Initializecorrespondences
  c.clear();
  int last_best = -1;
  const int n = trans_points.size();
  const int m = old_points.size();

  //Do for each point
  for(int i = 0; i < n; ++i)
  {  
    int best = 0;
    int second_best = 0;
    float best_dist = maxfloat;
    float second_best_dist = maxfloat;

    int starting_point = 0;
    if(last_best >= 0)
    {
      starting_point = last_best + 1;
    }
    else
    {
      for(int j = 0; j < m; ++j)
      {
        if(abs(trans_points[i].theta - old_points[j].theta) < abs(trans_points[i].theta - old_points[starting_point].theta))
        {
          starting_point = j;
        }
      }
    }

    int up = starting_point + 1;
    int down = starting_point;
    double last_dist_up = maxdouble;
    double last_dist_down = maxdouble;
    bool up_stopped = false;
    bool down_stopped = false;
    // While not stopped
    while(!(up_stopped && down_stopped))
    {
      bool now_up = !up_stopped && (last_dist_up <= last_dist_down);
      // Upwards scan if we are going up
      if(now_up)
      {
        // if up is too large we terminate
        if(up >= m)
        {
          up_stopped = true;
          continue;
        }
        if (up < 0)
        {
          up = 0;
        }
        // get the point corresponding to up and the distance to our trans_point
        Point p_up = old_points[up];
        float distance = p_up.distToPoint2(&trans_points[i]);
        last_dist_up = distance;
        // if the distance is good, update our distances and points
        if(distance < best_dist)
        {
          second_best = best;
          best = up;

          second_best_dist = best_dist;
          best_dist = distance;
        }
        else if(distance < second_best_dist)
        {
          second_best = up;
          second_best_dist = distance;
        }

        // if we are going further from starting point then update by jump table
        if(up > starting_point)
        {
          float phi = p_up.theta - trans_points[i].theta;
          double min_dist_up = sin(phi) * sin(phi) * best_dist; // why sin^2 and not min_dist_up^2?

          if(min_dist_up > best_dist)
          {
            up_stopped = true;
            continue;
          }

          if(p_up.r < trans_points[i].r)
          {
            up = jump_table[up][UP_BIG];
          }
          else
          {
            up = jump_table[up][UP_SMALL];
          }
        }
        else // increment upwards
        {
          up++;
        }
      }
      else
      {
        // if down is too small we terminate
        if(down < 0)
        {
          down_stopped = true;
          continue;
        }
        if(down >= m)
        {
          down = m - 1;
        }
        // get the point corresponding to up and the distance to our trans_point
        Point p_down = old_points[down];
        float distance = p_down.distToPoint2(&trans_points[i]);
        last_dist_down = distance;
        // if the distance is good, update our distances and points
        if(distance < best_dist)
        {
          second_best = best;
          best = down;

          second_best_dist = best_dist;
          best_dist = distance;
        }
        else if(distance < second_best_dist)
        {
          second_best = down;
          second_best_dist = distance;
        }

        // if we are going further from starting point then update by jump table
        if(down < starting_point)
        {
          float phi = trans_points[i].theta - p_down.theta;
          double min_dist_down = sin(phi) * sin(phi) * best_dist;

          if(min_dist_down > best_dist)
          {
            down_stopped = true;
            continue;
          }

          if(p_down.r < trans_points[i].r)
          {
            down = jump_table[down][DOWN_BIG];
          }
          else
          {
            down = jump_table[down][DOWN_SMALL];
          }
        }
        else // decrement down
        {
          down--;
        }
      }
    }

    c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
    last_best = best;
  } // end of for i (trans)
}


void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
