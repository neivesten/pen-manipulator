//
// Function block implementing a circular movement from P1 to P3 while
// passing through P2.  The block calculates the centre of the circle and
// generates a velocity vector to follow the arc at a trapezoidal speed
// profile.
//
FUNCTION_BLOCK FB_CircularMove
VAR_INPUT
    P1 : POS2D;             // Start point
    P2 : POS2D;             // Midpoint on arc (used only to determine arc path, not a stop point)
    P3 : POS2D;             // End point
    MaxSpeed : REAL;        // Maximum path speed [mm/s]
    Accel : REAL;           // Angular acceleration [rad/s^2]
    t : REAL;               // Time since start of movement [s]
    DecelerateAtEnd : BOOL; // Whether to decelerate at the end of movement
END_VAR
VAR_IN_OUT
    SpeedX : REAL;          // Current velocity in X direction (initial value on first call)
    SpeedY : REAL;          // Current velocity in Y direction (initial value on first call)
END_VAR
VAR
    CenterX, CenterY : REAL;
    Radius : REAL;
    StartAngle, EndAngle : REAL;
    CW : BOOL;

    ArcLength : REAL;
    Dir : REAL;
    DeltaAngle : REAL;
    t_acc : REAL;
    s_acc : REAL;
    s_const : REAL;
    t_const : REAL;
    t_total : REAL;
    s_t : REAL;
    v : REAL;
    v0 : REAL;
    Angle : REAL;
    angleP2 : REAL;
    AngularSpeed : REAL;
    t_dec : REAL;

    // Flags for special cases
    fullCircle : BOOL;
    cross : REAL;

    // Variables for circle center calculation using perpendicular bisectors
    dx12, dy12, dx23, dy23 : REAL;
    M12X, M12Y, M23X, M23Y : REAL;
    RHS1, RHS2, D : REAL;
END_VAR

// Main calculation block: Move from P1 to P3 through P2 along a circular arc

// Calculate circle center using perpendicular bisector intersection
dx12 := P2.X - P1.X;
dy12 := P2.Y - P1.Y;
dx23 := P3.X - P2.X;
dy23 := P3.Y - P2.Y;

// Mid‑points of the two chords
M12X := (P1.X + P2.X) * 0.5;
M12Y := (P1.Y + P2.Y) * 0.5;
M23X := (P2.X + P3.X) * 0.5;
M23Y := (P2.Y + P3.Y) * 0.5;

// Determinant for the 2×2 system
D := dx12 * dy23 - dy12 * dx23;

// Abort if points are collinear (determinant ≈ 0)
IF ABS(D) < 0.0001 THEN
    SpeedX := 0.0;
    SpeedY := 0.0;
    RETURN;
END_IF;

// Right‑hand sides of the linear equations
RHS1 := dx12 * M12X + dy12 * M12Y;
RHS2 := dx23 * M23X + dy23 * M23Y;

// Intersection of the two perpendicular bisectors → circle centre
CenterX := (RHS1 * dy23 - dy12 * RHS2) / D;
CenterY := (dx12 * RHS2 - RHS1 * dx23) / D;

Radius := SQRT((P1.X - CenterX)*(P1.X - CenterX) + (P1.Y - CenterY)*(P1.Y - CenterY));
StartAngle := FU_atan2(P1.Y - CenterY, P1.X - CenterX);
EndAngle := FU_atan2(P3.Y - CenterY, P3.X - CenterX);

angleP2 := FU_atan2(P2.Y - CenterY, P2.X - CenterX);

// Detect a full circle when start and end points coincide
fullCircle := ABS(P1.X - P3.X) < 0.0001 AND ABS(P1.Y - P3.Y) < 0.0001;

IF fullCircle THEN
    // Orientation based on cross product of radius vectors
    cross := (P1.X - CenterX) * (P2.Y - CenterY) - (P1.Y - CenterY) * (P2.X - CenterX);
    IF cross < 0 THEN
        CW := TRUE;
    ELSE
        CW := FALSE;
    END_IF;
ELSE
    // Determine clockwise or counterclockwise direction based on arc passing through P2
    IF ((StartAngle < EndAngle) AND (angleP2 > StartAngle) AND (angleP2 < EndAngle)) OR
       ((StartAngle > EndAngle) AND NOT (angleP2 > EndAngle AND angleP2 < StartAngle)) THEN
        CW := FALSE;
    ELSE
        CW := TRUE;
    END_IF;
END_IF;

// Set direction multiplier: -1 for clockwise, +1 for counterclockwise
IF CW = TRUE THEN
        Dir := -1.0;
ELSE
        Dir := 1.0;
END_IF

    // Magnitude of initial velocity provided via SpeedX/SpeedY
    v0 := SQRT(SpeedX * SpeedX + SpeedY * SpeedY);

// Adjust DeltaAngle for proper arc direction considering CW/CCW
IF fullCircle THEN
    DeltaAngle := 2 * 3.1415926;
ELSE
    DeltaAngle := EndAngle - StartAngle;
    IF DeltaAngle < 0 AND NOT CW THEN
        DeltaAngle := DeltaAngle + 2 * 3.1415926;
    ELSIF DeltaAngle > 0 AND CW THEN
        DeltaAngle := DeltaAngle - 2 * 3.1415926;
    END_IF;
    DeltaAngle := ABS(DeltaAngle);
END_IF;
ArcLength := Radius * DeltaAngle;
t_acc := (MaxSpeed - v0) / Accel;
IF t_acc < 0 THEN
    t_acc := 0.0;
END_IF;
s_acc := v0 * t_acc + 0.5 * Accel * t_acc * t_acc;
t_dec := MaxSpeed / Accel;
s_const := ArcLength - s_acc - (0.5 * MaxSpeed * t_dec);

IF s_const < 0 THEN
    // Triangular profile starting from v0
    v := SQRT(v0 * v0 + 2 * Accel * ArcLength);
    t_acc := (v - v0) / Accel;
    t_dec := v / Accel;
    t_const := 0.0;
    t_total := t_acc + t_dec;
ELSE
    t_const := s_const / MaxSpeed;
    t_total := t_acc + t_const + t_dec;
END_IF;

// Acceleration phase
IF t < t_acc THEN
    v := v0 + Accel * t;
    s_t := v0 * t + 0.5 * Accel * t * t;

// Constant speed phase or skipping deceleration
ELSIF t < t_acc + t_const OR NOT DecelerateAtEnd THEN
    v := MaxSpeed;
    s_t := s_acc + v * (t - t_acc);

// Deceleration phase
ELSIF t < t_total THEN
    t_dec := t - t_acc - t_const;
    v := MaxSpeed - Accel * t_dec;
    s_t := s_acc + s_const + MaxSpeed * t_dec - 0.5 * Accel * t_dec * t_dec;

// Motion complete - stop
ELSE
    v := 0.0;
    s_t := ArcLength;
END_IF;

Angle := StartAngle + Dir * (s_t / Radius);
AngularSpeed := v / Radius;
SpeedX := -SIN(Angle) * AngularSpeed * Radius;
SpeedY := COS(Angle) * AngularSpeed * Radius;
// End of FB_CircularMove
