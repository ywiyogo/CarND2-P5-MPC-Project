#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"   // for Eigen inverse()
#include "MPC.h"
#include "json.hpp"
#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
  return M_PI;
}
double deg2rad(double x)
{
  return x * pi() / 180;
}
double rad2deg(double x)
{
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if(found_null != string::npos) {
    return "";
  } else if(b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for(int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for(int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for(int j = 0; j < xvals.size(); j++) {
    for(int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if(sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if(s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if(event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
//          Eigen::VectorXd eig_ptsx(ptsx.data());
//          Eigen::VectorXd eig_ptsy(ptsy.data());
          
          double m_px = j[1]["x"];
          double m_py = j[1]["y"];
          double psi =  j[1]["psi"];
          double v = j[1]["speed"];
          

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          
          // Create the transformation matrix for mapping 2 CS
          // The transformation matrix consists rotation and translation to the vehicle position
          Eigen::Matrix3d T;
          
          T << cos(psi), -sin(psi), m_px,
               sin(psi), cos(psi), m_py,
               0,0,1;
          
          //
          vector<double> next_x_vals(ptsx.size());
          vector<double> next_y_vals(ptsx.size());
          
          // Transform the waypoints from map to vehicle CS
          unsigned int wp_len = ptsx.size();
          
          std::cout << "Length of ptsx: " << wp_len << std::endl;
          //std::cout << "T: \n" << T << std::endl;
          //std::cout << "T.inverse \n " << T.inverse() << std::endl;
          
          for (unsigned int i = 0; i < wp_len; i++ )
          {
            Eigen::Vector3d M;
            M << ptsx[i], ptsy[i], 1;
            Eigen::Vector3d V = T.inverse()*M;
            next_x_vals[i] = V[0];
            next_y_vals[i] = V[1];
          }
          Eigen::Vector3d Mp;
          Mp << m_px, m_py, 1;
          //Eigen::Vector3d Vp = T.inverse()*Mp;
          //std::cout << "Vp " << Vp <<std::endl;
          
          // px, px in vehicle CS
          //double v_px = Vp[0];
          //double v_py = Vp[1];
          
          // convert vector double of vehicle CS to Eigen::Vector
          double* p_ptsx = &next_x_vals[0];
          double* p_ptsy = &next_y_vals[0];
          Eigen::Map<Eigen::VectorXd> eig_ptsx(p_ptsx, wp_len);
          Eigen::Map<Eigen::VectorXd> eig_ptsy(p_ptsy, wp_len);
          
          double steer_value;
          double throttle_value;
  
          // 1. Fit the polynomial to the above points
          auto coeffs = polyfit(eig_ptsx, eig_ptsy, 3); // using third order to deal with the curve

          // 2. calculate cost function CTE and EPSI
          double cte = polyeval(coeffs, 0.);
          double epsi = -atan(coeffs[1]);
          
          // 3. Update the controller based on the CTE
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;
          
          // MPC Solve return a vector of actuators delta, and acceleration
          MpcSolution mpc_sol = mpc.Solve(state, coeffs);
          double scale_factor = 0.436332;  //25 deg = 0.436332 rad
          
          //std::cout << "DEBUG3, rad: " <<vars[0]<<" " << vars[1]<<" " << vars[2]<<" " << vars[3]<<" " << vars[4]<<" " << vars[5]<< std::endl;
          
          //4. Assigning steervalue
          // on the steering value scale, "left" is negative, while in the vehicle model, "left" is positive. 
          // So when the optimizer outputs a positive delta angle, that means the car is supposed to steer left. 
          // But in order to tell the simulator to steer left, you need to give it a negative value.
          // The simulator takes as input values in [-1, 1], where -1 is equivalent to 25 degrees to the left and 1 is equivalent to 25 degrees to the right.

          steer_value = -1 * mpc_sol.delta / scale_factor;
          throttle_value = mpc_sol.a;
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << cte << "\t" << steer_value << "\t" << throttle_value << std::endl;
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          
          msgJson["mpc_x"] = mpc_sol.xpts; //mpc_x_vals;
          msgJson["mpc_y"] = mpc_sol.ypts; //mpc_y_vals;

          // Display the waypoints/reference line
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
  h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if(req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection(
      [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if(h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
