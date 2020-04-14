# QuadrotorSwarmsSimulink
A simple matlab and simulink interface for the versatile control of quadrotor swarms, incorporating full quadrotor dynamics.

## Overview:
There are two primary Simulink diagrams:
  1) ErskineSingleDrone: This is the structure of a single drone, taking a control input, performing the control, and outputting the state

  2) ErskineMultiDrone: This simulates one or more swarms composed of the preceeding single drone type. Multiple swarms with the same initial conditions can be used to compare controllers

Both simulink diagrams must be initialized with their own launch files. As this model was developped for the simulation of quadrotor swarms in order to evaluate new algorithms, versatility and readability were prioritized over compile time and run time.

## Single Drone Model
This model is initialized by the SingleQuadInit.m script. It computes the forward dynamic model of the quadrotor given the net thrust $f$ and the body frame moments $\boldsymbol{\tau}$:

$\begin{align} \dot{\bf p} &= {\bf v} \\
\dot{\bf v} &= \frac{{\bf R}({\bf q})}{m}\begin{bmatrix}0\\0\\f \end{bmatrix}+ {\bf g} \\
\dot{\bf q} &= -\frac{1}{2} \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} \otimes {\bf q}  \\
\dot{\boldsymbol{\omega}} &= {\bf I}^{-1}\left( \boldsymbol{\tau} - \boldsymbol{\omega} \times {\bf I} \boldsymbol{\omega}  \right)
\end{align}$

where $\otimes$ is the quaternion multiplication operator. The thrust and moments are calculated as a function of the motor speeds using the coefficients of torque and thrust as shown in *Six D., Briot S., Erskine J., Chriette A. "Identification of the Propeller Coefficients and Dynamic Parameters of a Hovering Quadrotor from Flight Data", IEEE Robotics and Automation Letters, 5(2), 2020*.

In reality, the motor speeds of a quadrotors are controlled by an electronic speed controller (ESC), but for the sake of keeping this model simple, the propeller speeds $\boldsymbol{\omega}_p$ are controlled by the first order system

$\begin{align}
\dot{\boldsymbol{\omega}}_p = k_m (\boldsymbol{\omega}_p^* - \boldsymbol{\omega}_p)
\end{align}$

While quadrotor control often makes use of slow translational controllers and fast attitude controllers, for the sake of keeping a compact and simple diagram, here they are calculate both at once every loop. The controller can be selected by clicking on the drone block, and the types of control are listed below:

1) Position control

2) Velocity control

3) Acceleration control

4) Thrust Attitude control

5) Thrust Attitude Rate control

6) Thrust Moment control

Each controller outputs $[f, \boldsymbol{\tau}]$, which is used to to calculate the propeller reference speed $\boldsymbol{\omega}_p^{*}$. Thrust saturation is performed as required. All control is calculated by the *Control/SingleDrone/droneControl.m* function.

Drone parameters are set by the *SimSetup/setDefaultDroneParams.m* functions, and are organized as:
  * param.mechanical
    * m
    * l
    * I
  * param.motors
    * $\omega_{max}$
    * $\omega_{min}$
    * $\omega_{0}$
    * gain
    * kt_prop
    * kd_prop
    * allocation_matrix
  param.control
    * position
      * kp
      * kd
    * velocity
      * kp
    * attitude
      * kp
      * kd
    * attitude_rate
      * kp
    * limits
      * thrust
        * max
        * min
  * param.sensor
    * position_noise
    * velocity_noise
    * attitude_noise
    * gyroscope_noise

## Drone Swarm Model

Quadrotor
