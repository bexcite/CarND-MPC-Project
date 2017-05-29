#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <chrono>

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

// StateHistory
// keeps track of latest `history_time` ms states
class StateHistory {
public:
  StateHistory(double history_time) {
    history_time_ = history_time;
  }
  void add(Eigen::VectorXd state) {

    if (state[6] > 1.0) {
      history_.clear();
      return;
    }

    // add to history
    history_.push_back(state);

    double dt = state[6];

    // update times
    for (int i = 0; i < history_.size() - 1; ++i) {
      history_[i][6] += dt;
    }

    // remove elements out of history_time_ window
    while (history_.size() > 0 && history_[0][6] > history_time_) {
      history_.erase(history_.begin());
    }

  }

  void addActions(double steer_value, double throttle_value) {
    if (history_.size() == 0) return;
    history_[history_.size() - 1][4] = steer_value;
    history_[history_.size() - 1][5] = throttle_value;
  }

  double averageDt() {
    if (history_.size() == 0) { return 0.0; }

    return history_[0][6]/history_.size();

  }

  int length() {
    return history_.size();
  }

private:
  double history_time_;
  std::vector<Eigen::VectorXd> history_;

};

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

  StateHistory state_history(2.0);

  int polyOrder = 2;
  double Lf = 2.67;

  auto prev_clk = std::chrono::high_resolution_clock::now();
  auto cycle_clk = std::chrono::high_resolution_clock::now();

  bool started = false;

  double cycle_time = 0.0;
  double cycle_dist = 0.0;
  double cycle_dist_est = 0.0;
  double cycle_speed = 0.0;
  double prev_speed = 0.0;
  double prev_throttle = 0.0;
  double prev_steer_value = 0.0;
  double cycle_px = 0.0;
  double cycle_py = 0.0;


  std::vector<double> prev_state(6, 0.0);

  h.onMessage([&mpc, &prev_clk, &cycle_clk, polyOrder, &started,
               &cycle_dist, &cycle_dist_est, &cycle_time, &prev_speed,
               &cycle_speed, &prev_throttle, &prev_state, &Lf, &state_history,
               &cycle_px, &cycle_py, &prev_steer_value](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
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

          auto clk = std::chrono::high_resolution_clock::now();
          const double dt = std::chrono::duration<double>(clk - prev_clk).count();
          prev_clk = clk;

          double delta_dist;
          double delta_dist_est;
          double delta_speed;
          double delta_speed_est;

          if (!started || dt > 1.0) {
            cycle_clk = std::chrono::high_resolution_clock::now();
            cycle_dist = 0.0;
            cycle_dist_est = 0.0;
            cycle_time = 0.0;
            prev_speed = 0.0;
            prev_throttle = 0.0;
            cycle_speed = 0.0;
            cycle_px = px;
            cycle_py = py;
            started = true;
          } else {
            cycle_time = std::chrono::duration<double>(clk - cycle_clk).count();


          }


          // Testing model params (to find out the real value of correct physics in "our world"

          delta_dist = 1.60934 * prev_speed * dt/3600.0;
          cycle_dist += delta_dist;

          delta_speed = v - prev_speed;
          prev_speed = v;

          delta_speed_est = 24000.0 * prev_throttle * dt/3600.0;

          cycle_speed += delta_speed_est;

          delta_dist_est = 1.60934 * cycle_speed * dt/3600.0;
          cycle_dist_est += delta_dist_est;

          double diffx = px - cycle_px;
          double diffy = py - cycle_py;
          double cycle_dist_xy = sqrt(diffx * diffx + diffy * diffy) / 1000.0;


          std::cout << "DT = " << dt << std::endl;
          std::cout << "SPEED_DELTA     = " << delta_speed << std::endl;
          std::cout << "SPEED_DELTA_EST = " << delta_speed_est << std::endl;
          std::cout << "C_DIST_DELTA     = " << delta_dist << std::endl;
          std::cout << "C_DIST_DELTA_EST = " << delta_dist_est << std::endl;
          std::cout << "C_DIST     = " << cycle_dist << std::endl;
          std::cout << "C_DIST_EST = " << cycle_dist_est << std::endl;
          std::cout << "C_DIST_XY  = " << cycle_dist_xy << std::endl;

          // <<<<<< end of testing




          // Estimate current state
          double cpx = prev_state[0] + 1.60934 * prev_state[3] * cos(prev_state[2]) * dt / 3600.0;
          double cpy = prev_state[1] - 1.60934 * prev_state[3] * sin(prev_state[2]) * dt / 3600.0;
          double cpsi = prev_state[2] - 1000.0 * prev_state[3] * prev_steer_value / (Lf * 1.60934) * dt / 3600.0;
          double cv = prev_state[3] + 28000.0 * prev_throttle * dt / 3600.0;

          double d_psi = 1000.0 * prev_state[3] * (prev_steer_value / (Lf * 1.60934)) * dt / 3600.0;

          std::cout << "PX  = " << px << std::endl;
          std::cout << "CPX = " << cpx << std::endl;
          std::cout << "PY  = " << py << std::endl;
          std::cout << "CPY = " << cpy << std::endl;
          std::cout << "PSI  = " << psi << std::endl;
          std::cout << "CPSI = " << cpsi << std::endl;
          std::cout << "dPSI = " << d_psi << std::endl;
          std::cout << "V  = " << v << std::endl;
          std::cout << "CV = " << cv << std::endl;

          double throttle_calc = (v - prev_state[3]) / (dt / 3600.0);
          std::cout << "THROTTLE_CALC  = " << throttle_calc << std::endl;



          // Make current state vector
          Eigen::VectorXd state(7);
          state << px, py, psi, v, 0.0, 0.0, dt;

          state_history.add(state);

          std::cout << "AVERAGE_DT = " <<  state_history.averageDt() << std::endl;
          std::cout << "history_length = " << state_history.length() << std::endl;


          // Find coeffs of reference line

          // Convert ptsx and ptsy to car coordinates
          // Transform matrix
          // [cos(psi), -sin(psi), px ]
          // [sin(psi),  cos(psi), py ]
          // [       0,         0,  1 ]

          // Inverse of transformation matrix
          // [ cos(psi), sin(psi), - (  px * cos(psi) + py * sin(psi)) ]    [ ptsx[0] ]
          // [-sin(psi), cos(psi), - (- px * sin(psi) + py * cos(psi)) ]  * [ ptsy[0] ]
          // [        0,        0,                                   1 ]    [       1 ]
          int pts_size = ptsx.size();
          for (int i = 0; i < pts_size; ++i) {
            double ptsxi = ptsx[i];
            double ptsyi = ptsy[i];
            ptsx[i] = ptsxi * cos(psi) + ptsyi * sin(psi)
                      - (px * cos(psi) + py * sin(psi));
            ptsy[i] = - 1.0 *  ptsxi * sin(psi) + ptsyi * cos(psi)
                      + px * sin(psi) - py * cos(psi);
//            std::cout << "ptsx[" << i << "] = " << ptsyi << std::endl;
//            std::cout << "sin(psi) = " << sin(psi) << std::endl;
//            std::cout << - 1.0 *  ptsxi * sin(psi) << std::endl;
//            std::cout << ptsyi * cos(psi) << std::endl;
//            std::cout << px * sin(psi) << std::endl;
//            std::cout << - py * cos(psi) << std::endl;
          }

          // Calculate coeffs
          Eigen::VectorXd ptsx_v = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsy_v = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

          auto coeffs = polyfit(ptsx_v, ptsy_v, polyOrder);






          json msgJson;



          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

//          msgJson["mpc_x"] = mpc_x_vals;
//          msgJson["mpc_y"] = mpc_y_vals;


          // Convert ptsx and ptsy to car coordinates
          // Transform matrix
          // [cos(psi), -sin(psi), px ]
          // [sin(psi),  cos(psi), py ]
          // [       0,         0,  1 ]

          // Inverse of transformation matrix
          // [ cos(psi), sin(psi), - (  px * cos(psi) + py * sin(psi)) ]    [ ptsx[0] ]
          // [-sin(psi), cos(psi), - (- px * sin(psi) + py * cos(psi)) ]  * [ ptsy[0] ]
          // [        0,        0,                                   1 ]    [       1 ]

          // Test ptsx transformation
          // "ptsx":{-9.93,11.95,34.12,53.85,74.31,96.88},
          // "ptsy":{19.48,42.54,66.68,88.81,112.49,139.38}

//          ptsx = {10.0, 20.0};
//          ptsy = {0.0, 0.0};
//          px = 5.0;
//          py = 0.0;
//          psi = 1.5707963268;
//
//          std::cout << "pos: " << px << ", " << py << ", " << psi << std::endl;




//          json jtemp;
//          jtemp["ptsx"] = ptsx;
//          jtemp["ptsy"] = ptsy;
//          std::cout << "transf: " << jtemp.dump() << std::endl;


/*
          mpc_x_vals.resize(pts_size);
          mpc_y_vals.resize(pts_size);
          for (int i = 0; i < pts_size; ++i) {
            mpc_x_vals[i] = ptsx[i];
            mpc_y_vals[i] = polyeval(coeffs, ptsx[i]);
//            std::cout << "mpc_xy_vals[" << i << "] = ("
//                      << mpc_x_vals[i] << ", " << mpc_y_vals[i] << ")" << std::endl;
          }


//          jtemp.clear();
//          jtemp["mpc_x_vals"] = mpc_x_vals;
//          jtemp["mpc_y_vals"] = mpc_y_vals;
//          std::cout << "polyfit line: " << jtemp.dump() << std::endl;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
*/


          //Display the waypoints/reference line
//          vector<double> next_x_vals = {10.0, 20.0, 100.0, 200.0, 600.0};
//          vector<double> next_y_vals = {0.0, 0.0, 0.0, 0.0, 0.0};
          vector<double> next_x_vals; // = ptsx;
          vector<double> next_y_vals; // = ptsy;
//          vector<double> next_x_vals = {-9.93,11.95,34.12,53.85,74.31,96.88};
//          vector<double> next_y_vals = {19.48,42.54,66.68,88.81,112.49,139.38};
//          vector<double> next_x_vals = {9.93,11.95,34.12,53.85,74.31,96.88};
//          vector<double> next_y_vals = {0.48,0.54,0.68,0.81,0.49,0.38};


                  //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          next_x_vals.resize(pts_size);
          next_y_vals.resize(pts_size);
          for (int i = 0; i < pts_size; ++i) {
            next_x_vals[i] = ptsx[i];
            next_y_vals[i] = polyeval(coeffs, ptsx[i]);
//            std::cout << "mpc_xy_vals[" << i << "] = ("
//                      << mpc_x_vals[i] << ", " << mpc_y_vals[i] << ")" << std::endl;
          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          // Calculate current CTE
          double cte = polyeval(coeffs, 0); // expected - current (y = 0)
          double epsi = - atan(coeffs[1] + coeffs[2] * 0.0); // mul 0.0 left for clarity here x = 0.0
          std::cout << "CTE  = " << cte << std::endl;
          std::cout << "EPSI = " << epsi << std::endl;


          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = 0.6; // * cte;
          double throttle_value = 0.2;



          // Initialize start state (we calculate everything in car coordinate)
          Eigen::VectorXd state_mpc(6);
          state_mpc << 0.0, 0.0, 0.0, v, cte, epsi;

          // MPC Solve
          auto vars = mpc.Solve(state_mpc, coeffs, state_history.averageDt());

          size_t nn = size_t(vars[0]);
          size_t x_start = 1;
          size_t y_start = x_start + nn;
          size_t psi_start = y_start + nn;
          size_t v_start = psi_start + nn;
          size_t cte_start = v_start + nn;
          size_t epsi_start = cte_start + nn;
          size_t delta_start = epsi_start + nn;
          size_t a_start = delta_start + nn - 1;



          std::cout << "XXS = " << std::fixed << std::setprecision(4);
          for (int i = x_start; i < y_start; ++i) {
            std::cout << vars[i] << ", ";
          }
          std::cout << std::endl;

          std::cout << "YYS = " << std::fixed << std::setprecision(4);
          for (int i = y_start; i < psi_start; ++i) {
            std::cout << vars[i] << ", ";
          }
          std::cout << std::endl;

          std::cout << "PSIS = " << std::fixed << std::setprecision(4);
          for (int i = psi_start; i < v_start; ++i) {
            std::cout << vars[i] << ", ";
          }
          std::cout << std::endl;

          std::cout << "VVS = " << std::fixed << std::setprecision(4);
          for (int i = v_start; i < cte_start; ++i) {
            std::cout << vars[i] << ", ";
          }
          std::cout << std::endl;

          std::cout << "CTES = " << std::fixed << std::setprecision(4);
          for (int i = cte_start; i < epsi_start; ++i) {
            std::cout << vars[i] << ", ";
          }
          std::cout << std::endl;

          std::cout << "EPSI = " << std::fixed << std::setprecision(4);
          for (int i = epsi_start; i < delta_start; ++i) {
            std::cout << vars[i] << ", ";
          }
          std::cout << std::endl;

          std::cout << "DELTAS = " << std::fixed << std::setprecision(4);
          for (int i = delta_start; i < a_start; ++i) {
            std::cout << vars[i] << ", ";
          }
          std::cout << std::endl;

          std::cout << "AAS = " << std::fixed << std::setprecision(4);
          for (int i = a_start; i < vars.size(); ++i) {
            std::cout << vars[i] << ", ";
          }
          std::cout << std::endl;


          mpc_x_vals.clear();
          mpc_y_vals.clear();
          mpc_x_vals.resize(nn);
          mpc_y_vals.resize(nn);
          for (int i = 0; i < nn; ++i) {
            mpc_x_vals[i] = vars[x_start + i];
            mpc_y_vals[i] = vars[y_start + i];
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

//          std::cout << "x1   = " << vars[0] << std::endl;
//          std::cout << "y1   = " << vars[1] << std::endl;
//          std::cout << "psi1 = " << vars[2] << std::endl;
//          std::cout << "v1   = " << vars[3] << std::endl;
//          std::cout << "delta = " << vars[6] << std::endl;
//          std::cout << "a     = " << vars[7] << std::endl;

          steer_value = vars[delta_start]; // -0.012; //6043625087635; // vars[6];
          throttle_value = vars[a_start]; //vars[7];



          std::cout << "STEER VALUE = " << steer_value << std::endl;


          // Save prev state
          prev_state[0] = px;
          prev_state[1] = py;
          prev_state[2] = psi;
          prev_state[3] = v;
          prev_state[4] = steer_value;
          prev_state[5] = throttle_value;

          prev_throttle = throttle_value;
          prev_steer_value = steer_value;



          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          state_history.addActions(steer_value, throttle_value);


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
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  /*
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
   */


  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
