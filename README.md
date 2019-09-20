[//]: # (Image References)
 

[image1]: ./support/PID_Compensation_Animated.gif
[image2]: ./support/controler_output_formula.jpg
[image3]: ./support/proportional_term_formula.jpg
[image4]: ./support/Integral_term_formula.jpg
[image5]: ./support/Derivative_term_formula.jpg 
[image6]: ./support/pseudocode_twiddle.jpg 
[image7]: ./support/cubic_Spline.jpg
[image8]: ./support/Cost_schematic_chart.jpg
[image9]: ./support/Simulator_lap.jpg


## Eduardo Ribeiro de Campos - September 2019


# **CarND-Controls-PID** 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

To conclude the project is necessary reach the goals according to the project [rubric](https://review.udacity.com/#!/rubrics/1972/view).

The input README File from udacity with a lot of instructions about the project is [here](./Udacity_README.md)


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


## Tunning the parameters:

To tune the Parameter Kp , Ki and Kd it was chosen in this project use the sistem presented by [Sebastian Thrun](https://en.wikipedia.org/wiki/Sebastian_Thrun) in the SDCND - term 2 - class13.13.Twiddle.
    
[![alt text][image6]](https://www.youtube.com/watch?v=2uQ2BSzDvXs)
