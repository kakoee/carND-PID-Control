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


double normalize(double input)
{
	if(input>1) return 1;
	if(input<-1) return -1;
	else return input;

    const double min = -1000;
    const double max = 1000;
    double average      = (min + max) / 2;
    double range        = (max - min) / 2;
    double normalized_x = (input - average) / range;
    return normalized_x;
}



int main()
{
  uWS::Hub h;

  PID pid;
  PID pid_th;
  // TODO: Initialize the pid variable.
  

// first Manual tuning to find out the good values. then use twiddle to tweak further
/*
1- Set all gains to zero.
2- Increase the P gain until the response to a disturbance is steady oscillation.
3- Increase the D gain until the the oscillations go away (i.e. it's critically damped).
4- Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
5- Set P and D to the last stable values.
6- Increase the I gain until it brings us to the setpoint with the number of oscillations desired 

note that, If the oscillations grow bigger and bigger then you need to reduce P gain. 
*/

  pid.Init(0.125, 0.002, 3.2);
  pid_th.Init(0.28, 0, 0.02);

  //pid.twiddle=true;
  //pid_th.twiddle=true;


  h.onMessage([&pid,&pid_th](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value,throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
                  

		  pid.UpdateError(cte);
          steer_value = normalize(pid.TotalError());

          pid_th.UpdateError(fabs(cte));     
          throttle_value = pid_th.TotalError() ;

          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout   << " coeff steering:"<< pid.Kp << ", "<< pid.Ki << ", " << pid.Kd << std::endl;
          //std::cout << " coeff throttle:"<< pid_th.Kp << ", "<< pid_th.Ki << ", " << pid_th.Kd << std::endl;


          //std::cout << "AddSub: " << pid.add_sub << "TotalError: " << pid.total_error << " BestError: " << pid.best_err << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;// + throttle_value;
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
