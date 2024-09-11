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



### Filter

1.Constructor (Public Section)

The constructor initializes the alpha smoothing factor, and it sets the initial value of prevOutput to 0. This ensures that the first output starts from 0 or some baseline value.
```
SoftStart(double alpha) {
    this->alpha = alpha;
    prevOutput = 0;
}
```

This constructor is simple but important. It initializes the filter with a specific smoothing factor and prepares the output state for use by setting prevOutput to an initial value of 0.

2. **filter** function
   The filter function is the heart of the class, where the actual smoothing happens.
```
double filter(double input) {
    return alpha * input + (1 - alpha) * prevOutput;
}
```
Input: The current input value that needs to be filtered.
Output: The filtered output, which is a weighted combination of the new input and the previous output.

The formula:
```
output = alpha * input + (1 - alpha) * prevOutput;
```
This is a weighted average of the current input and the previous output. Let's break it down:
-alpha * input: This term gives weight to the new input. If alpha is large (close to 1), the new input will have a stronger influence on the output. If alpha is small (close to 0), the new input will have a smaller influence.
-(1 - alpha) * prevOutput: This term gives weight to the previous output. The larger the value of 1 - alpha, the more influence the previous output has on the new output, making the change more gradual.
-Combined Result: By combining these two terms, the output changes gradually from the old value (prevOutput) to the new value (input). Each time the filter function is called, the new output will be stored in prevOutput (implicitly, as it replaces the current output). This creates a smooth transition between successive inputs.
