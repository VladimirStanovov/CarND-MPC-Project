# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Description

The Model Predictive Control (MPC) consists in generating a control signal derived from an optimized trajectory of the object movement for a number of steps in time. In this project, the object to control is the vehicle in a simulator, running around the track.

The state of the vehicle is described with position (x,y), angle psi and velocity v. The kinematic model is used to predict the next state of the vehicle, updating all the variables of the state. Controlling the vehicle involves using the actuators – steering angle delta and throttle value. The update equations also use actuator values, i. e. for example, the velocity is influenced by throttle, and pis is influenced by steering angle delta. The actuator values are limited: the throttle is limited by interval [-1,1], while the delta is limited by interval [-0.436332, 436322], which is around 25 degrees both sides. So, controlling the vehicle means setting the appropriate throttle and steering values for every step of the control.

The model of vehicle movement contains four update equations:

x[t+1] = x[t] + v*cos(psi)*dt,
y[t+1] = y[t] + v*sin(psi)*dt,
psi[t+1] = psi[t] - v*delta/Lf*dt,
v[t+1] = v[t] + a*dt,

where Lf is the length from front to center of gravity of the vehicle.

MPC is based on predicting the trajectory of the vehicle movement, which is performed by building an optimized trajectory. The optimization problem cost function consists in calculating the cross track error, angle error, velocity error, as well as actuator values and gaps between consequent actuations, which should be minimized. One of the most important problems here is the tuning of weight coefficients assigned to every part of the cost function. Moreover, the optimization problem is characterized by a set of constraints, for example, the trajectory should start from the current vehicle position. Two other important parameters are the number of points N of the trajectory to be predicted in the future, and the time step size dt between these points.

After implementing the basic algorithm, which also included converting the reference line from global to local coordinates, and adding the polynomial (I used the 3rd order) into the model, the next step was to set the parameters values, so that the vehicle would be able to drive safely with the reference speed of 40.

First of all, I tried different number of steps, going down from 25, as it was in the quizzes down to 12-16 which worked a little better. So, the idea here is that there is no need to predict so far in time, probably because the state of the vehicle will change very much over this time. As for the dt value, as I did all experiments with latency, I could not get it running well with the dt = 0.1, so I ended up with 0.05. The latency is actually a huge problem, because the immediate control value that is generated from the trajectory built is not actual anymore, at the moment when it is sent (100ms has passed!). So, in my opinion, a nice solution to this is to use the control value, that is 100ms later than, so that we predict the vehicle state in 100ms, and take the control value that is valid at that moment. In my implementation, as I returned the whole set of control values, I took the 3rd control value for both delta and a (considering that the dt value is 0.05, 100 ms is 2*dt, that is the 3rd value). Probably, and even better solution would be to use the previous control values as constraints in our model to represent that for the next 100ms the control signals will be fixed to these values. Unfortunately, I didn’t implement this idea to see it working.

The next experiments were conducted with the reference speed of 63, which resulted in around 60mph actual speed (depends on the weight). Running at such speed was only possible when I decreased the number of points N to 10, because with values larger than 12 the prediction was too far away to the future, and was not useful. By the way, setting N lower than 8 resulted to large oscillations and vehicle going off the track, because the prediction trajectory was too short to stabilize the vehicle. 

Although tuning N and dt be effective, there is a way to improve the algorithm by tuning the cost function.  In particular, it is important to have the difference between consequent actuations quite small, especially considering the steering angle. As a result of several experiments, I figured out that setting the weight for this part of the cost function to values from 200 to 800 is a good idea, as it make the control much smoother. As for the value of the steering angle itself, I set the weight to 0.05, i.e. a very small weight, because in my opinion, it is not a problem to have a large steering angle, as long as we came to it smoothly. As for the throttle value, I set the weight equal to 10, so that the vehicle would not accelerate too fast, and wouldn’t overshoot the reference velocity and brake all the time. One of the most important weights is the angle error weight – tuning it helped passing the whole track at 60mph. First I tried setting to 5, 10, and it gave significant improvement in tracking the reference trajectory. As a result, I ended up with the value of 40, as increasing it further didn’t give any improvements. As for the weight if the cross track error, changing it didn’t give any obvious improvements, so I left it equal to 1. The velocity weight is similar to the P value in PID controller, to setting it to 5 simply resulted in smaller difference between the reference speed and the actual speed of the vehicle.

The latency dealing approach described above may look fair, but it is actually not strictly correct. The reason is that we generate the trajectory exactly from the position of the vehicle in time and space, whereas we won't be able to control it for the next 100ms. So, the control values in the first couple of iterations (considering dt = 0.05 and latency = 0.1, it is exactly 2 iterations) are generated, but never executed, so actually the vehicle appears in a different postion, than predicted by the trajectory. In the simulator, for the next 100ms the vehicle will execute the previous command sent from the control algorithm, so the correct solution should consider the beginning of predicted trajectory from the point where the vehicle will be at that moment. This could be implemented by using the same update equations, which describe the vehicle dynamics, presented above.

The update equations should consider the initial x and y position as 0, as well as psi, as we are in the vehicle coordinate system, while the velocity v is the same in both global and local coordinate system. The update equations should be executed using previous actuator values, saved from the previous step, and the number of iterations to execute should be the same as the number of steps of the model that suit into 100ms (2 steps in my case). After calculating the new position of the vehicle, the state is initialized with these new values, and not with zeros, as before. When running the simulator, one may observe that in this case the green line representing the predicted trajectory starts little bit in front of the vehicle.

Changing the latency handling method influenced the behaviour of the whole algorithm, so I had to tune the parameters once again. The maximum velocity that I was able to achieve was 70mph, with the following parameters:

cte weight: 1,
epsi weight: 40,
v weight: 5,
delta weight: 10,
a weight: 10,
sequential delta weight: 3000,
sequential a weight: 1.

The following two videos show the vehicle running at 40mph:

and 70mph:
