#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "memory.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

class State
{
  public:
    static int lane_;
    double max_vel_, acc_;
    int id_;

    virtual void calcNewSpeed(double &ref_vel) = 0;

    vector<int> next_valid_states_;

    std::tuple<double, double>
    getClosestCar(int prev_size, double car_s, const std::vector<std::vector<double>> &sensor_fusion)
    {
      double vel = 49;
      double dist = 1000;
      for (auto different_car : sensor_fusion)
      {
        float d = different_car[6];
        if ((d < 2 + 4 * lane_ + 2) && (d > 2 + 4 * lane_ - 2))
        { //each line is 4 meters if it's in my lane
          double oc_speed_x = different_car[3];
          double oc_speed_y = different_car[4];
          double check_speed = sqrt(oc_speed_x * oc_speed_x + oc_speed_y * oc_speed_y);
          double check_car_s = different_car[5];
          check_car_s += (double) prev_size * 0.02 * check_speed;
          if (check_car_s > car_s && check_car_s - car_s < dist)
          {
            dist = check_car_s - car_s;
            vel = check_speed;
          }
        }
      }
      return std::make_tuple(dist, vel * 2.236936); //m/s to mph
    }

    bool isLaneFree(int prev_size, double car_s, const std::vector<std::vector<double>> &sensor_fusion)
    {
      for (auto different_car : sensor_fusion)
      {
        float d = different_car[6];
        if ((d < 2 + 4 * lane_ + 2) && (d > 2 + 4 * lane_ - 2))
        { //each line is 4 meters if it's in my lane
          double oc_speed_x = different_car[3];
          double oc_speed_y = different_car[4];
          double check_speed = sqrt(oc_speed_x * oc_speed_x + oc_speed_y * oc_speed_y);
          double check_car_s = different_car[5];
          check_car_s += (double) prev_size * 0.02 * check_speed;
          if ((check_car_s > car_s) && (check_car_s - car_s < 30))
          {
            max_vel_ = check_speed * 2.236936;
            return false;
          }

        }
      }
      return true;
    }

    bool
    isLaneFreeOvertake(int lane, int prev_size, double car_s, const std::vector<std::vector<double>> &sensor_fusion)
    {
      for (auto different_car : sensor_fusion)
      {
        float d = different_car[6];
        if ((d < 2 + 4 * lane + 2) && (d > 2 + 4 * lane - 2))
        { //each line is 4 meters if it's in my lane
          double oc_speed_x = different_car[3];
          double oc_speed_y = different_car[4];
          double check_speed = sqrt(oc_speed_x * oc_speed_x + oc_speed_y * oc_speed_y);
          double check_car_s = different_car[5];
          check_car_s += (double) prev_size * 0.02 * check_speed;
          if ((check_car_s - car_s > -10) && (check_car_s - car_s < 45))
          {
            return false;
          }
        }
      }
      return true;
    }


};

int State::lane_ = 1;

class GoStraight : public State
{
  public:
    GoStraight()
    {
      id_ = 0;
      next_valid_states_ = {0, 1, 2};
      max_vel_ = 49;
      acc_ = 0.224;
    }

    virtual void calcNewSpeed(double &ref_vel)
    {
      if (ref_vel < max_vel_) ref_vel += acc_;
    }
};

class PrepareTurn : public State
{
  public:
    PrepareTurn(double max_vel)
    {
      id_ = 2;
      next_valid_states_ = {1, 2};

      max_vel_ = max_vel;
      acc_ = 0.224;
    }

    virtual void calcNewSpeed(double &ref_vel)
    {
      ref_vel = (ref_vel >= max_vel_) ? ref_vel - acc_ : ref_vel + acc_;
    }
};

class Turn : public State
{
  public:
    Turn()
    {
      id_ = 1;
      next_valid_states_ = {0, 1};

      max_vel_ = 49;
      acc_ = 0.124;
    }

    virtual void calcNewSpeed(double &ref_vel)
    {
      if (ref_vel < max_vel_) ref_vel += acc_;
    }

};


void changeState(int prev_size, double car_s, double car_d, const std::vector<std::vector<double>> &sensor_fusion,
                 std::shared_ptr<State> &car_state)
{
  auto next_valid_states = car_state->next_valid_states_;
  int lane = car_state->lane_;

  if (std::find(next_valid_states.begin(), next_valid_states.end(), 0) != next_valid_states.end())
  {
    if ((car_d < 2 + 4 * lane + 2) && (car_d > 2 + 4 * lane - 2))
    { //if car is on its lane
      if (car_state->isLaneFree(prev_size, car_s, sensor_fusion))
      {
        if (car_state->id_ != 0) car_state.reset(new GoStraight());
        return;
      }
    }
  }
  if (std::find(next_valid_states.begin(), next_valid_states.end(), 1) != next_valid_states.end())
  {
    if (car_state->id_ == 1) return;
    bool lane_changed = false;
    if (lane == 0)
    {
      if (car_state->isLaneFreeOvertake(1, prev_size, car_s, sensor_fusion))
      {
        lane = 1;
        lane_changed = true;
      }
    }
    else if (lane == 1)
    {
      if (car_state->isLaneFreeOvertake(2, prev_size, car_s, sensor_fusion))
      {
        lane = 2;
        lane_changed = true;
      }
      else if (car_state->isLaneFreeOvertake(0, prev_size, car_s, sensor_fusion))
      {
        lane = 0;
        lane_changed = true;
      }
    }
    else
    {
      if (car_state->isLaneFreeOvertake(1, prev_size, car_s, sensor_fusion))
      {
        lane = 1;
        lane_changed = true;
      }
    }
    if (lane_changed)
    {
      car_state->lane_ = lane;
      car_state.reset(new Turn());
      return;
    }

  }

  if (std::find(next_valid_states.begin(), next_valid_states.end(), 2) != next_valid_states.end())
  {
    if (car_state->id_ != 2)
    { car_state.reset(new PrepareTurn(car_state->max_vel_)); }
    else
    {
      auto closest_car = car_state->getClosestCar(prev_size, car_s, sensor_fusion);
      if (std::get<0>(closest_car) > 120)
      { car_state.reset(new GoStraight()); }
      else
      { if(std::get<0>(closest_car)<20)
        car_state->max_vel_ = std::get<1>(closest_car)/2; //slow down even more when we are too close
        else
        car_state->max_vel_ = std::get<1>(closest_car); }
    }
  }
}


int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  double ref_vel = 0;
  std::shared_ptr<State> car_state = std::shared_ptr<State>(new GoStraight());
  car_state->lane_ = 1;
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                      &map_waypoints_dx, &map_waypoints_dy, &ref_vel, &car_state]
                      (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {

                  auto s = hasData(data);

                  if (s != "")
                  {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];

                      // Previous path data given to the Planner
                      auto previous_path_x = j[1]["previous_path_x"];
                      auto previous_path_y = j[1]["previous_path_y"];
                      // Previous path's end s and d values
                      double end_path_s = j[1]["end_path_s"];
                      double end_path_d = j[1]["end_path_d"];

                      // Sensor Fusion Data, a list of all other cars on the same side
                      //   of the road.
                      auto sensor_fusion = j[1]["sensor_fusion"];

                      json msgJson;
                      int prev_size = previous_path_x.size();


                      if (prev_size > 0)
                      {
                        car_s = end_path_s;
                      }

                      changeState(prev_size, car_s, car_d, sensor_fusion, car_state);

                      car_state->calcNewSpeed(ref_vel);


                      vector<double> ptsx, ptsy;
                      double ref_x = car_x;
                      double ref_y = car_y;
                      double ref_yaw = deg2rad(car_yaw);

                      if (prev_size < 2) //If the previous path is empty or almost empty we need to calculate two tangent points
                      {
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                      }
                      else
                      {
                        //If we have previous point we will start plotting the path at the end of old path
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];

                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                      }

                      //Next points are waypoint that are approx 30, 60 and 90 m away from the car
                      auto next_wp0 = getXY(car_s + 30, (2 + 4 * car_state->lane_), map_waypoints_s, map_waypoints_x,
                                            map_waypoints_y);
                      auto next_wp1 = getXY(car_s + 60, (2 + 4 * car_state->lane_), map_waypoints_s, map_waypoints_x,
                                            map_waypoints_y);
                      auto next_wp2 = getXY(car_s + 90, (2 + 4 * car_state->lane_), map_waypoints_s, map_waypoints_x,
                                            map_waypoints_y);

                      ptsx.push_back(next_wp0[0]);
                      ptsx.push_back(next_wp1[0]);
                      ptsx.push_back(next_wp2[0]);

                      ptsy.push_back(next_wp0[1]);
                      ptsy.push_back(next_wp1[1]);
                      ptsy.push_back(next_wp2[1]);

                      //Shift to local car coordinates. Origin car is 0 angle
                      for (int i = 0; i < ptsx.size(); i++)
                      {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                      }

                      tk::spline s;
                      s.set_points(ptsx, ptsy);
                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      //We are using previous path to ensure the smooth transition
                      for (int i = 0; i < previous_path_x.size(); i++)
                      {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                      }


                      double target_x = 30;
                      double target_y = s(target_x);

                      double target_dist = sqrt(target_x * target_x + target_y * target_y);
                      double x_add_on = 0;

                      double N = target_dist / (0.02 * ref_vel / 2.24); // 2.24-> mph to m/sec 0.02 -> timestamp
                      double step = target_x / N;
                      for (int i = 1; i <= 50 - previous_path_x.size(); i++)
                      {
                        double x_car = x_add_on + step;
                        double y_car = s(x_car);
                        x_add_on = x_car;

                        //come back to map coordinate from car coordinates
                        double x_map = x_car * cos(ref_yaw) - y_car * sin(ref_yaw);
                        double y_map = x_car * sin(ref_yaw) + y_car * cos(ref_yaw);
                        x_map += ref_x;
                        y_map += ref_y;

                        next_x_vals.push_back(x_map);
                        next_y_vals.push_back(y_map);
                      }

                      msgJson["next_x"] = next_x_vals;
                      msgJson["next_y"] = next_y_vals;

                      auto msg = "42[\"control\"," + msgJson.dump() + "]";

                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    }  // end "telemetry" if
                  }
                  else
                  {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                }  // end websocket if
              }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                 {
                   std::cout << "Connected!!!" << std::endl;
                 });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
                    {
                      ws.close();
                      std::cout << "Disconnected" << std::endl;
                    });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
