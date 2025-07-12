//
// Simple PI controller used by the motion blocks. Computes an output
// velocity based on the position error. Output and integral are
// clamped to the provided limit to avoid windup.
//
FUNCTION_BLOCK FB_PIController
VAR_INPUT
    Setpoint : REAL;  // Desired value
    Measurement : REAL; // Current value
    Kp : REAL;        // Proportional gain
    Ki : REAL;        // Integral gain
    Ts : REAL;        // Cycle time [s]
    Limit : REAL;     // Maximum absolute output
END_VAR
VAR_IN_OUT
    Integral : REAL;  // Integrator state
END_VAR
VAR_OUTPUT
    Out : REAL;       // Controller output
END_VAR
VAR
    Error : REAL;
END_VAR

Error := Setpoint - Measurement;
Integral := Integral + Ki * Error * Ts;

// Clamp integral term
IF Integral > Limit THEN
    Integral := Limit;
ELSIF Integral < -Limit THEN
    Integral := -Limit;
END_IF;

Out := Kp * Error + Integral;

// Clamp controller output
IF Out > Limit THEN
    Out := Limit;
ELSIF Out < -Limit THEN
    Out := -Limit;
END_IF;

// End of FB_PIController
