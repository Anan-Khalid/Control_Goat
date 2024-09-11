# Control_Goat

## Implementation

### PID Control
Constructor (Public Section)
The constructor initializes the PID controller with user-specified gains (kp, ki, kd), and sets the initial values of prevError and integral to 0. This is important for maintaining consistency when calculating the PID output.
```
PID(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    prevError = 0;
    integral = 0;
}
```
**compute** function calculates the control output based on the current error and the PID formula.
1. Calculate the error:
```
double error = setpoint - input;
```
setpoint: The desired value you want to achieve.
input: The current measured value (feedback from the system).

2. Update the integral term:
   ```
   integral += error;
```
Accumulates the error over time. This term helps eliminate steady-state error by increasing the controller output if the error persists over time.

3. Compute the derivative:
```
double derivative = error - prevError;
```
This calculates the rate of change of the error (how fast the error is changing). It helps to dampen the system by anticipating future changes based on the current rate of error change.

4. Update prevError:
```
prevError = error;
```
This prepares the prevError for the next time the function is called, so the derivative term can be calculated properly in future steps.

5. Return the PID control output:
```
return kp * error + ki * integral + kd * derivative;
```
The output is a weighted sum of the proportional, integral, and derivative terms. This output can then be used to drive an actuator or modify the system's behavior.
The output is a weighted sum of the proportional, integral, and derivative terms. This output can then be used to drive an actuator or modify the system's behavior.
