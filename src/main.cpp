#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>

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


std::vector<double> p = {0.14,0.0009,3.5};
std::vector<double> dp = {0.01,0.0001,0.1};
int idx = 0;
int twiddle_count = 0;
bool twiddle_on = false;
double best_error = 10000000;
double error_;

void Twiddle(PID &pid_control){
  error_ = pow(pid_control.TotalError(),2.0);
  p[idx] += dp[idx];

  std::cout<<error_<<std::endl;
  if(error_<best_error){
    best_error = error_;
    dp[idx] *= 1.1;
    idx = (idx+1) % 3 ;
    std::cout<<"1st!!!"<<std::endl;
  }else{
    p[idx] -= 2 * dp[idx];
    error_ = pow(pid_control.TotalError(),2.0);
    if(error_<best_error){
      best_error = error_;
      dp[idx] *= 1.1;
      idx = (idx+1) % 3;
      std::cout<<"2nd!!!!"<<std::endl;
    }else{
      p[idx] += dp[idx];
      dp[idx] *= 0.9;
      idx = (idx+1) % 3;
      std::cout<<"3rd!!!!!!!"<<std::endl;
    }
  }

  pid_control.Init(p[0],p[1],p[2]);

}

int main()
{
  uWS::Hub h;

  PID pid;
  PID pid_speed;
  // TODO: Initialize the pid variable.
  //pid.Init(0.05,0.006,4.5);
  //PI control pid.Init(0.05,0.006,0) is not good(go out of the track )
  //PD control pid.Init(0.05,0.0,4.5) runs the a bit right side of the road. and rot good at the turn;
  //PID controlpid.Init(0.14,0.0009,3.5) best so far
  pid.Init(0.13,0.0009,3.5);
  pid_speed.Init(0.15,0.00012,1.3);



  h.onMessage([&pid, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          double max_steering_angle = 1.0;
          pid.UpdateError(cte);
          pid_speed.UpdateSpeedError(speed,45.0);

          steer_value = pid.TotalError();
          throttle = pid_speed.TotalError();
          if (throttle >1.0) throttle =1.0;

          if (steer_value > max_steering_angle) steer_value = max_steering_angle;
          if (steer_value < -max_steering_angle) steer_value = -max_steering_angle;

          // DEBUG

          if(!twiddle_on){
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          //std::cout << "P: " << pid.p_error << "\t I: " << pid.i_error << "\t D: " << pid.d_error  << std::endl;
          //std::cout << "P_term: " << 0.13*pid.p_error << "\t I_term: " << 0.009*pid.i_error << "\t D_term: " <<  3.5*pid.d_error << std::endl;
          //std::cout << "P_term: " << 0.15*pid_speed.p_error << "\t I_term: " << 0.00012*pid_speed.i_error << "\t D_term: " << 1.5*pid_speed.d_error  << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }else{
            twiddle_count+=1;
            if(twiddle_count<1000){
              Twiddle(pid);
              std::cout<<twiddle_count<<std::endl;
              std::cout<<"dp[0]"<<dp[0]<<"dp[1]"<<dp[1]<<"dp[2]"<<dp[2]<<std::endl;
              std::cout<<"p[0]"<<p[0]<<"p[1]"<<p[1]<<"p[2]"<<p[2]<<std::endl;
              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.5;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }else{
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.5;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }
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