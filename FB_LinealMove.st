//
// Function block implementing a trapezoidal speed profile for linear
// movements.  The block accelerates from the start position, maintains a
// constant speed and optionally decelerates at the end.
//
FUNCTION_BLOCK FB_LinealMove
VAR_INPUT
    StartPos : POS2D;   // Starting position
    TargetPos : POS2D;  // Target position
    MaxSpeed : REAL;    // Maximum speed during movement
    Accel : REAL;       // Acceleration used for ramp-up and ramp-down
    t : REAL;           // Elapsed time since start of movement
    DecelerateAtEnd : BOOL; // Whether to ramp down at the end
END_VAR
VAR_OUTPUT
    SpeedX : REAL;      // Current speed in the X direction
    SpeedY : REAL;      // Current speed in the Y direction
END_VAR
VAR
    TotalDist : REAL;   // Total distance between start and target
    DirX : REAL;        // Normalized direction vector X component
    DirY : REAL;        // Normalized direction vector Y component
    t_acc : REAL;       // Time needed to accelerate to MaxSpeed
    s_acc : REAL;       // Distance covered during acceleration phase
    s_const : REAL;     // Distance covered at constant MaxSpeed
    t_const : REAL;     // Time spent at constant MaxSpeed
    t_total : REAL;     // Total duration of the entire move
    v : REAL;           // Current magnitude of the velocity vector
END_VAR

// Compute direction and total distance
TotalDist := SQRT(EXPT(TargetPos.X - StartPos.X,2) + EXPT(TargetPos.Y - StartPos.Y,2));
IF TotalDist = 0 THEN
    SpeedX := 0;
    SpeedY := 0;
ELSE
    DirX := (TargetPos.X - StartPos.X) / TotalDist;
    DirY := (TargetPos.Y - StartPos.Y) / TotalDist;

    t_acc := MaxSpeed / Accel;
    s_acc := 0.5 * Accel * t_acc * t_acc;
    s_const := TotalDist - 2 * s_acc;
    IF s_const < 0 THEN
        // Triangular profile
        t_acc := SQRT(TotalDist / Accel);
        t_const := 0;
        t_total := 2 * t_acc;
    ELSE
        // Trapezoidal profile
        t_const := s_const / MaxSpeed;
        t_total := 2 * t_acc + t_const;
    END_IF;

    // Determine current speed magnitude
    IF t < t_acc THEN
        v := Accel * t;
    ELSIF t < t_acc + t_const OR NOT DecelerateAtEnd THEN
        v := MaxSpeed;
    ELSIF t < t_total THEN
        v := MaxSpeed - Accel * (t - t_acc - t_const);
    ELSE
        v := 0;
    END_IF;

    // Apply speed along direction
    SpeedX := v * DirX;
    SpeedY := v * DirY;
END_IF;

// End of FB_LinealMove
END_FUNCTION_BLOCK

