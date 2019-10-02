[//]: # (Image References)
 
[image1]: ./support/PID_Compensation_Animated.gif
[image2]: ./support/controler_output_formula.jpg
[image3]: ./support/proportional_term_formula.jpg
[image4]: ./support/Integral_term_formula.jpg
[image5]: ./support/Derivative_term_formula.jpg 
[image6]: ./support/pseudocode_twiddle.jpg 
[image7]: ./support/Twiddle_algorithm.jpg
[image8]: ./support/twiddle_video.jpg
[image9]: ./support/KP_parameter_Decimal_Analysis.jpg
[image10]: ./support/Kd_parameter_Combined_Kp_0_1_scaled.jpg
[image11]: ./support/Kd_parameter_Combined_Kp_0_2_scaled.jpg
[image12]: ./support/Comparison_Fixed_Kd_scaled.jpg
[image13]: ./support/Ki_parameter_Combined_Kp_0_2_Kd_6_scaled.jpg
[image14]: ./support/Ki_influence_scaled.jpg
[image15]: ./support/Final_lap.jpg


## Eduardo Ribeiro de Campos - October 2019


# **CarND-Controls-PID** 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

To conclude the project is necessary reach the goals according to the project [rubric](https://review.udacity.com/#!/rubrics/1972/view).

The input README File from udacity with a lot of instructions about the project is [here](./Udacity_README.md)


Here is link to [main.cpp](./src/main.cpp) file.


External references:

https://pid-tuner.com/pid-control/<br/>
https://en.wikipedia.org/wiki/PID_controller



## Project overview.
In this project you'll revisit the lake race track from the Behavioral Cloning Project. This time, however, you'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

One more thing. The speed limit has been increased from 30 mph to 100 mph. Get ready to channel your inner Vin Diesel and try to drive SAFELY as fast as possible! NOTE: you don't have to meet a minimum speed to pass.


## About PID 

The PID controller (an abbreviation of Proportional Integral Differential) is the most widely applied feedback control formula/algorithm. It is applied in a huge variety of 'things' to automate them, such as planes, drones, cars, coffeemakers, wind turbines, furnaces, and manufacturing units. It is fair to say that the PID controller is the work horse for automation. 


### The effect of P, I and D:

The PID Controller has three parameters that should be tuned. The proportional term (Kp), the integral term (Ki) and the differential term (Kd). The animation below shows the effects of varying Kp and Ki.

![alt text][image1]


## PID controller theory

The PID control scheme is named after its three correcting terms, whose sum constitutes the manipulated variable (MV). The proportional, integral, and derivative terms are summed to calculate the output of the PID controller. Defining u(t) as the controller output, the final form of the PID algorithm is:

![alt text][image2]

where:

- **Kp** - is the proportional gain, a tuning parameter,<br/>
- **Ki** - is the integral gain, a tuning parameter,<br/>
- **Kd** - is the derivative gain, a tuning parameter,<br/>
- **e(t) = SP - PV(t)** - is the error (SP is the setpoint, and PV(t) is the process variable),<br/>
- **t** - is the time or instantaneous time (the present),<br/>
- **tau** - is the variable of integration (takes on values from time 0 to the present t.<br/>

#### Proportional term:

The proportional term produces an output value that is proportional to the current error value. The proportional response can be adjusted by multiplying the error by a constant Kp, called the proportional gain constant.
The proportional term is given by:

![alt text][image3]

#### Integral term:

The contribution from the integral term is proportional to both the magnitude of the error and the duration of the error. The integral in a PID controller is the sum of the instantaneous error over time and gives the accumulated offset that should have been corrected previously. The accumulated error is then multiplied by the integral gain (Ki) and added to the controller output.The integral term is given by:

![alt text][image4]

#### Derivative term:

The derivative of the process error is calculated by determining the slope of the error over time and multiplying this rate of change by the derivative gain Kd. The magnitude of the contribution of the derivative term to the overall control action is termed the derivative gain, Kd.The derivative term is given by:

![alt text][image5]



## Defining the parameters Kp , Ki and Kd.


### -Kp Parameter.

In the next graphic shows the bevavior os the cte (cross track error) changing the decimal value from -0.4 to +0.4

![alt text][image9]

The values of `Kp = 0.1` and `Kp = 0.2` presented the best results.



### - Kd Parameter.

It was checked the values from 0 to 6 in 2 scenarios , scenario 1 with Kp =0.1 and scenario 2 with Kp =0.2.
The `Y` axis was scaled to understand the influence of each value.

####  Scenario 1 .:  Kp = 0.1.

![alt text][image10]

The values of `Kd = 6` presented the best results.


####  Scenario 2 .:  Kp = 0.2.


![alt text][image11]


Following the scenario 1, the values of `Kd = 6` presented the best results.


#### Comparing  scenarios. ( Best pair for Kp and Kd).


![alt text][image12]

The pair `Kp = 0.2` and `Kd = 6` presented the best results ( closest to de Zero in the `Y` axis).


### - Ki Parameter.

With `Kp` and `Ki` already defined, It was checked 5 condition changing de decimal value to verify the influence of this parameter.

![alt text][image13]


The values of `Ki = 0.001` presented the best results.



### The influence of each parameter.


In the next graphic, It was plotted the influence of each parameter to the answer of the controller.


![alt text][image14]

The result shows exactly the same behavior expected from the theoretical concept.

All these graphics was constructed using the `python` code and the libraries `Pandas` e `Matplotlib`.
To see the code click [Here](./support/Parameters_analysis.ipynb) to acess a `Jupyter Notebook file`.


## Tunning the parameters:

To tune the Parameter Kp , Ki and Kd it was chosen in this project, use the sistem presented by [Sebastian Thrun](https://en.wikipedia.org/wiki/Sebastian_Thrun) in the SDCND - term 2 - class13.13.Twiddle function.
    
[![alt text][image6]](https://www.youtube.com/watch?v=2uQ2BSzDvXs)


The picture below shows schematically the algorithm used to fine tuning each parameter interacting with the simulator.

It was defined as  "N", the variable of the total time-steps used to adjust each parameter.In the first part (N/2), it was checked the condition p[i] += dp[i]. In the second part , it was checked the second condition  p[i] -= 2* dp[i]. After run this algorithm thru these 3 parameter (Kp , Ki and Kd) , the output will be the best parameter adjusted according to the error calculated and de Delta value (dp) also adjusted to dp[i]*= 1.1 at the cases where the error is lower than the current best error, or dp[i]*= 0.9 at the cases where the error is greater than the current best error.

![alt text][image7]


## Starting the Twiddle Algorithm.

To compile the code it is necessary execute the basic build Instructions.

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./pid`.

But if we must run a code to execute the twiddle algoritm, is necessary apply the parameter in the item 3

the step is: Run it: `./pid Kp Ki Kd`, in our case `./pid 0.2 0.001 6`

The video below shows a loop with 2 runs in order to check 2 cases for the same parameter. The main point to be observed here is the time-step size defined as 1000. This number ensure that the simaltion will provide the constante error (vehicle cross track error) in 2 scenarios, at straight line and curvature.

[![alt text][image8]](https://youtu.be/ZXUpi8LupC4)

The "N" variable could present different results according to the system performance.
All this repport and results presented was run on the `Udacity workspace` enviroment with a `GPU` support.


The [code](./src/main.cpp) gives an output file named: [total_summary.txt](./support/total_summary.txt) with the total report from the twiddle process. It is possible to see in the next information only the first 2 iterations. Take note that at the end of each iteration has a summary showing the partial value for each parameter, the sum of the delta values (dp) and the best error.

```

********************************************************************
ITERATION: 1
	For Kp = 0.2; Delta_Kp = 0.01
		case1-> Kp = 0.21; error = 20.0359
		case2-> Kp = 0.19; error = 20.0328
-----------------------------------------------------------
Partial Kp = 0.19  Partial delta Kp = 0.011  Best error = 20.0328
-----------------------------------------------------------
	For Ki = 0.001; Delta_Ki = 0.0001
		case1-> Ki = 0.0011; error = 0.0213983
		case2-> Ki = 0.0009; error = 0.0247221
-----------------------------------------------------------
Partial Ki = 0.0011  Partial delta Ki = 0.00011  Best error = 0.0213983
-----------------------------------------------------------
	For Kd = 6; Delta_Kd = 0.1
		case1-> Kd = 6.1; error = 0.0206721
		case2-> Kd = 5.9; error = 0.0251339
-----------------------------------------------------------
Partial Kd = 6.1  Partial delta Kd = 0.11  Best error = 0.0206721
-----------------------------------------------------------
summarizing iteration 1    Partial Best error = 0.0206721 for: 
partial KP 0.19    Partial Ki = 0.0011    Partial Kd = 6.1
the sum of delta is: 0.12111 and the tolerance is: 0.1
********************************************************************
ITERATION: 2
	For Kp = 0.19; Delta_Kp = 0.011
		case1-> Kp = 0.201; error = 0.0185696
		case2-> Kp = 0.179; error = 0.0247209
-----------------------------------------------------------
Partial Kp = 0.201  Partial delta Kp = 0.0121  Best error = 0.0185696
-----------------------------------------------------------
	For Ki = 0.0011; Delta_Ki = 0.00011
		case1-> Ki = 0.00121; error = 0.0238381
		case2-> Ki = 0.00099; error = 0.0223933
-----------------------------------------------------------
Partial Ki = 0.0011  Partial delta Ki = 9.9e-05  Best error = 0.0185696
-----------------------------------------------------------
	For Kd = 6.1; Delta_Kd = 0.11
		case1-> Kd = 6.21; error = 0.0221108
		case2-> Kd = 5.99; error = 0.023214
-----------------------------------------------------------
Partial Kd = 6.1  Partial delta Kd = 0.099  Best error = 0.0185696
-----------------------------------------------------------
summarizing iteration 2    Partial Best error = 0.0185696 for: 
partial KP 0.201    Partial Ki = 0.0011    Partial Kd = 6.1
the sum of delta is: 0.111199 and the tolerance is: 0.1
********************************************************************


```

## Final Result.

After 7 iteration the final parameter is:

```
Final_Kp = 0.201  Final_Ki = 0.0010901  Final_Kd = 6.29701
```

To compile the code without the twiddle function, it is necessary execute the basic build Instructions.

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./pid`.


The next picture is a link to show de video with the official Lap.


[![alt text][image15]](https://youtu.be/acjbPkAqUtk)



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).
