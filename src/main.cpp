#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  static std::vector<double> p = {0,0,0};
  static std::vector<double> dp = {1,1,1};
  pid.Init(0.134611, 0.000270736, 3.05349); //found from a PID example online for PID testing https://github.com/jeremy-shannon/CarND-PID-Control-Project/blob/master/src/main.cpp
  // pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          static bool twiddle = false;
          static bool first = true;
          static int i = 0;
          static int step = 0;
          static double tol = 0.2;
          if (twiddle){
            static double best_err = std::numeric_limits<double>::max();
            pid.itter++;
            if (dp[0] + dp[1] + dp[2] > tol){
              if (pid.itter%pid.n*2 == 0){
                if (first){
                  best_err = pid.TotalError();
                }
                if (step == 0){
                  p[i%3] += dp[i%3];
                  pid.Init(p[0], p[1], p[2]);
                  step = 1;
                  pid.itter = 0;
                }
                else if (step == 1){
                  double err = pid.TotalError();
                  if (err < best_err){
                    best_err = err;
                    dp[i%3] *= 1.1;
                  }
                  else{
                    p[i%3] -= 3*dp[i%3];
                    pid.Init(p[0], p[1], p[2]);
                    step = 2;
                    pid.itter = 0;
                  }
                }
                else if (step == 2){
                  double err = pid.TotalError();
                  if (err < best_err){
                    best_err = err;
                    dp[i%3] *= 1.1;
                  }
                  else{
                    p[i%3] += dp[i%3];
                    dp[i%3] *= 0.9;
                  }
                  i++;
                }
              }
              pid.UpdateError(cte);
              std::cout << p[0] << "," << p[1] << "," << p[2] << std::endl;
            }
            else{
              // std::cout << p[0] << "," << p[1] << "," << p[2] << std::endl;
              return 0;
            }
          }
          static double sum_cte = 0;
          static double last_cte = 0;
          if (first){
            last_cte = cte;
            first = false;
          }
          sum_cte += cte;
          double p = pid.Kp * cte;
          double ii = pid.Ki * sum_cte;
          double d = pid.Kd * (cte-last_cte);
          steer_value = -p-ii-d;
          last_cte = cte;
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
