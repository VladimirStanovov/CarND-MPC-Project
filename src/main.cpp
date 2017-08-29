#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          auto pts_len = ptsx.size();
          Eigen::VectorXd ptsx_e(pts_len);
          Eigen::VectorXd ptsy_e(pts_len);
          for(unsigned i=0;i!=pts_len;i++)
          {
            ptsx_e(i) = cos(psi)*(ptsx[i]-px) + sin(psi)*(ptsy[i]-py);
            ptsy_e(i) = -sin(psi)*(ptsx[i]-px) + cos(psi)*(ptsy[i]-py);
          }

          double lat_x = 0;
          double lat_y = 0;
          double lat_psi = 0;
          double lat_v = v;
          double dt = 0.05; //also should be changed in MPC.cpp
          const double Lf = 2.67;

          for(unsigned int i = 0; i < 2; i++) { // 2 is because we have 2 steps of the model with dt=0.05
            lat_x = lat_x + lat_v*cos(lat_psi)*dt;
            lat_y = lat_y + lat_v*sin(lat_psi)*dt;
            lat_psi = lat_psi - lat_v/Lf*mpc.prevDelta*dt;
            lat_v = lat_v + mpc.prevA*dt;
          }

          auto coeffs = polyfit(ptsx_e, ptsy_e, 3);
          double cte = polyeval(coeffs, lat_x) - lat_y;
          double epsi = -atan(coeffs[1] + coeffs[2] * lat_x + coeffs[3] * lat_x * lat_x);

          Eigen::VectorXd state(6);
          //state << 0, 0, 0, v, cte, epsi; //car coordinates - x,y,psi = 0
          state << lat_x, lat_y, lat_psi, lat_v, cte, epsi; //car coordinates - x,y,psi = 0

          double steer_value;
          double throttle_value;
          auto solution = mpc.Solve(state, coeffs);
          size_t N = solution[0];
          std::vector<double> x_vals;
          std::vector<double> y_vals;
          std::vector<double> delta_vals;
          std::vector<double> a_vals;
          for(unsigned i = 0; i!=N;i++)
          {
            if(i!=0)
            {
              x_vals.push_back(solution[i+1]);
              y_vals.push_back(solution[N+i+1]);
            }
            delta_vals.push_back(solution[N*2+i+1]);
            a_vals.push_back(solution[N*2+N-1+i+1]);
          }
          steer_value = delta_vals[0]/0.436332;
          throttle_value = a_vals[0];

          mpc.prevDelta = delta_vals[0];
          mpc.prevA = a_vals[0];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = x_vals;
          msgJson["mpc_y"] = y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (unsigned int i=0 ; i < pts_len; ++i) {
            next_x_vals.push_back(ptsx_e(i));
            next_y_vals.push_back(ptsy_e(i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
