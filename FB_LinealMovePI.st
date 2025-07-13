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
    TotalErr : REAL;
    NormErrX : REAL;
    NormErrY : REAL;
    Scale : REAL;
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

// Compute total error magnitude
TotalErr := SQRT(ErrX * ErrX + ErrY * ErrY);

// Avoid divide-by-zero
IF TotalErr > 0.01 THEN
    NormErrX := ErrX / TotalErr;
    NormErrY := ErrY / TotalErr;
ELSE
    NormErrX := 0.0;
    NormErrY := 0.0;
END_IF;

// Compute common scale from PI controller on the magnitude
Scale := Kp * TotalErr + (Ki * TotalErr * Ts);
IF Scale > MaxSpeed THEN
    Scale := MaxSpeed;
END_IF;

SpeedX := Scale * NormErrX;
SpeedY := Scale * NormErrY;

// End of FB_LinealMovePI
