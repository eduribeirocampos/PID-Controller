#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <fstream>

// for convenience
using nlohmann::json;
using std::string;

std::ofstream outfile ("total_summary.txt");
std::ofstream outfile_be ("best_error.txt");
std::ofstream outfile_kp ("kp.txt");
std::ofstream outfile_ki ("ki.txt");
std::ofstream outfile_kd ("kd.txt");
std::ofstream outfile_dp ("sum_dp.txt");


bool twiddle = false;
double best_error = 9999;

double error_case1 = 9999; //// to check p += dp
double error_case2 = 9999; //// to check p -= dp

double tolerance = 0.05;
int n = 1000;

char parameter;
double initial_parameter;
double initial_delta_parameter;


int it = 1;

std::vector<double> final_report{};

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[]) {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  
  double init_Kp , init_Ki , init_Kd;

  // result from twiddle  - Final_Kp = 0.2199  Final_Ki = 0.00051  Final_Kd = 2.95846

  if(argc != 4)
  {
    init_Kp = 0.2199;
    init_Ki = 0.00051;
    init_Kd = 2.95846;
  }
  else
  {
    init_Kp = atof(argv[1]);
    init_Ki = atof(argv[2]);
    init_Kd = atof(argv[3]);
    twiddle = true;
  }
   
 
 std::vector<double>p{init_Kp,init_Ki,init_Kd};
 std::vector<double>dp{0.01,0.0001,0.1};

 pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid,&n ,&best_error ,&tolerance,&error_case1 , &error_case2, &it ,&parameter,initial_parameter,&initial_delta_parameter,&p,&dp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    static int time_step = 0; 
    static int parameter_to_tune = 0;
    static double p_case1 = 0;
    static double p_case2 = 0;
    static double sum_dp = 9999999999999999;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          //double speed = std::stod(j[1]["speed"].get<string>());
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          time_step++;

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
                   
          if (twiddle == true)
          {                      
            if (sum_dp > tolerance)
            {              
              if (time_step == 1)
              {
                if(it == 1 && parameter_to_tune == 0)
                {
                  outfile << "********************************************************************" <<std::endl;
                  outfile << "ITERATION: " << it <<std::endl;
                }
                
                initial_parameter = p[parameter_to_tune];
                initial_delta_parameter = dp [parameter_to_tune];

                //checking case 1 -> p +=dp
                p[parameter_to_tune] += dp[parameter_to_tune];
                p_case1 = p[parameter_to_tune];
                pid.Init(p[0],p[1],p[2]);

                if(parameter_to_tune == 0)
                {
                  outfile<<"	For Kp = " << initial_parameter<<"; Delta_Kp = "<< initial_delta_parameter <<std::endl;
                }
                else if(parameter_to_tune == 1)
                {
                  outfile<<"	For Ki = " << initial_parameter<<"; Delta_Ki = "<< initial_delta_parameter <<std::endl;
                }
                if(parameter_to_tune == 2)
                {
                  outfile<<"	For Kd = " << initial_parameter<<"; Delta_Kd = "<< initial_delta_parameter <<std::endl;
                }


              }            
              else if (time_step > n/4 && time_step < n/2) 
              {
                //Calculating error case 1 
                error_case1 += pow(cte,2);
              }             
              else if (time_step == n/2  ) 
              {
                error_case1 /= (n/2);  
                
                if(parameter_to_tune == 0)
                {
                  outfile<<"		case1-> Kp = " << p_case1<<"; error = "<< error_case1 <<std::endl;
                  std::cout<<"Kp - case 1 - OK"<<std::endl;                    
               
                }
                else if(parameter_to_tune == 1)
                {
                  outfile<<"		case1-> Ki = " << p_case1<<"; error = "<< error_case1 <<std::endl;
                  std::cout<<"Ki - case 1 - OK"<<std::endl;                     
                }
                if(parameter_to_tune == 2)
                {
                  outfile<<"		case1-> Kd = " << p_case1<<"; error = "<< error_case1 <<std::endl;
                  std::cout<<"Kd - case 1 - OK"<<std::endl;                      
                }
                //Reset simulator
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

                //checking case 2 -> p -=dp 
                p[parameter_to_tune] -= 2* dp[parameter_to_tune];
                p_case2 = p[parameter_to_tune];
                pid.Init(p[0],p[1],p[2]);
  
              }             
              else if (time_step > (3*n)/4 && time_step < n  ) 
              {
              //Calculating error case 2 
                error_case2 += pow(cte,2);
              }
              else if (time_step == n) 
              {
                //calculating the mean of the errors
                 
                error_case2 /= (n/2); 
                if(parameter_to_tune == 0)
                {
                  outfile<<"		case2-> Kp = " << p_case2<<"; error = "<< error_case2 <<std::endl;
                  std::cout<<"Kp - case 2 - OK"<<std::endl;                      
                }
                else if(parameter_to_tune == 1)
                {
                  outfile<<"		case2-> Ki = " << p_case2<<"; error = "<< error_case2 <<std::endl;
                  std::cout<<"Ki - case 2 - OK"<<std::endl;                   
                }
                if(parameter_to_tune == 2)
                {
                  outfile<<"		case2-> Kd = " << p_case2<<"; error = "<< error_case2 <<std::endl;
                  std::cout<<"Kd - case 2 - OK"<<std::endl;                    
                }

                //defininig parameter and best error 
                if (error_case1 > best_error && error_case2 > best_error)
                {
                  dp[parameter_to_tune]*=0.9;
                  //fixing the parameter for the initial value
                  p[parameter_to_tune] = initial_parameter;

                  if(parameter_to_tune == 0)
                  {
                    pid.Init(initial_parameter,p[1],p[2]);
                  }
                  else if(parameter_to_tune == 1)
                  {
                    pid.Init(p[0],initial_parameter,p[2]);
                  }
                  if(parameter_to_tune == 2)
                  {
                    pid.Init(p[0],p[1],initial_parameter);
                  }
                }
                else
                {
                  dp[parameter_to_tune]*=1.1;
                  if(error_case1 < error_case2)
                  {
                    p[parameter_to_tune] = p_case1;
                    best_error = error_case1;


                  if(parameter_to_tune == 0)
                  {
                    pid.Init(p_case1,p[1],p[2]);
                  }
                  else if(parameter_to_tune == 1)
                  {
                    pid.Init(p[0],p_case1,p[2]);
                  }
                  if(parameter_to_tune == 2)
                  {
                    pid.Init(p[0],p[1],p_case1);
                  }
                  }
                  else
                  {
                    p[parameter_to_tune] = p_case2;
                    best_error = error_case2;
                  }
                }
                
                if(parameter_to_tune == 0)
                {
                  outfile<<"-----------------------------------------------------------"<< std::endl;
                  outfile<<"Partial Kp = " << p[parameter_to_tune] << "  Partial delta Kp = "<<dp[parameter_to_tune]<< "  Best error = " << best_error<<std::endl;
                  outfile<<"-----------------------------------------------------------"<< std::endl;
                }
                else if(parameter_to_tune == 1)
                {
                  outfile<<"-----------------------------------------------------------"<< std::endl;
                  outfile<<"Partial Ki = " << p[parameter_to_tune] << "  Partial delta Ki = "<<dp[parameter_to_tune]<< "  Best error = " << best_error<<std::endl;
                  outfile<<"-----------------------------------------------------------"<< std::endl;
                }
                if(parameter_to_tune == 2)
                {
                  outfile<<"-----------------------------------------------------------"<< std::endl;
                  outfile<<"Partial Kd = " << p[parameter_to_tune] << "  Partial delta Kd = "<<dp[parameter_to_tune]<< "  Best error = " << best_error<<std::endl;                  
                  outfile<<"-----------------------------------------------------------"<< std::endl;
                }
    
                //reseting variables
                time_step=0;
                sum_dp = dp[0]+dp[1]+dp[2];
                error_case1=0;
                error_case2=0;

                parameter_to_tune++;
                if (parameter_to_tune ==3)
                {
                  
                  outfile<<"summarizing iteration "<<it<<"    Partial Best error = "<< best_error <<" for: "<<std::endl;
                  outfile<<"partial KP "<<p[0]<<"    Partial Ki = "<< p[1] <<"    Partial Kd = "<< p[2]<<std::endl;
                  outfile<<"the sum of delta is: "<<sum_dp<<" and the tolerance is: "<< tolerance <<std::endl;
                  outfile_be<< it << ","<<best_error<<std::endl;
                  outfile_kp<< it << ","<<p[0]<<std::endl;
                  outfile_ki<< it << ","<<p[1]<<std::endl;
                  outfile_kd<< it << ","<<p[2]<<std::endl;
                  outfile_dp<< it << ","<<sum_dp<<std::endl;
                  

                  parameter_to_tune =0;
                  it++;
                  std::cout<<" ITERATION " <<it<< " OK  - the sum of dp is: "<<sum_dp<<std::endl;                  
                  std::cout << "********************************************************************" <<std::endl;                  
                  
                  outfile << "********************************************************************" <<std::endl;
                  outfile << "ITERATION: " << it <<std::endl;
                  
                  if (sum_dp < tolerance)
                  {
                    outfile<<"*********************************************************************" << std::endl;                    
                    outfile<<"Final_Kp = "<<p[0] <<"  Final_Ki = " <<p[1] <<"  Final_Kd = " <<p[2] << std::endl;
                    outfile.close();
                  }                  
                 
                }

                //Reset simulator
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);


              }
            }
          }
          
          // DEBUG
          //std::cout<<"it: " << time_step <<" CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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