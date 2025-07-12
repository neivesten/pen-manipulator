//
// Linear move implemented using two PI controllers operating on the
// X and Y axes. The block drives the TCP towards the provided target
// position and outputs velocity commands saturated to MaxSpeed.
//
FUNCTION_BLOCK FB_LinealMovePI
VAR_INPUT
    CurPos    : POS2D; // Current TCP position
    TargetPos : POS2D; // Target position
    MaxSpeed  : REAL;  // Maximum speed [mm/s]
    Kp        : REAL;  // Proportional gain
    Ki        : REAL;  // Integral gain
    Ts        : REAL;  // Cycle time [s]
END_VAR
VAR_IN_OUT
    SpeedX : REAL; // Current speed command X (state on call)
    SpeedY : REAL; // Current speed command Y (state on call)
END_VAR
VAR
    IntX : REAL;   // Integrator state for X axis
    IntY : REAL;   // Integrator state for Y axis
    ErrX : REAL;
    ErrY : REAL;
END_VAR

ErrX := TargetPos.X - CurPos.X;
ErrY := TargetPos.Y - CurPos.Y;

IntX := IntX + Ki * ErrX * Ts;
IntY := IntY + Ki * ErrY * Ts;

// Anti-windup limits
IF IntX > MaxSpeed THEN
    IntX := MaxSpeed;
ELSIF IntX < -MaxSpeed THEN
    IntX := -MaxSpeed;
END_IF;
IF IntY > MaxSpeed THEN
    IntY := MaxSpeed;
ELSIF IntY < -MaxSpeed THEN
    IntY := -MaxSpeed;
END_IF;

SpeedX := Kp * ErrX + IntX;
SpeedY := Kp * ErrY + IntY;

// Saturate outputs
IF SpeedX > MaxSpeed THEN
    SpeedX := MaxSpeed;
ELSIF SpeedX < -MaxSpeed THEN
    SpeedX := -MaxSpeed;
END_IF;
IF SpeedY > MaxSpeed THEN
    SpeedY := MaxSpeed;
ELSIF SpeedY < -MaxSpeed THEN
    SpeedY := -MaxSpeed;
END_IF;

// End of FB_LinealMovePI
