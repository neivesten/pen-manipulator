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
VAR_OUTPUT
    SpeedX : REAL;          // Current velocity in X direction
    SpeedY : REAL;          // Current velocity in Y direction
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
    Angle : REAL;
	angleP2 : REAL;
    AngularSpeed : REAL;
    t_dec : REAL;

    D : REAL;
    Ux, Uy : REAL;
    Vx, Vy : REAL;
    Wx, Wy : REAL;
	SELECT: INT;
END_VAR

// Main calculation block: Move from P1 to P3 through P2 along a circular arc

// Calculate circle center using perpendicular bisectors method
Ux := 0.5 * (P1.X + P2.X);
Uy := 0.5 * (P1.Y + P2.Y);
Vx := 0.5 * (P2.X + P3.X);
Vy := 0.5 * (P2.Y + P3.Y);

Wx := P2.X - P1.X;
Wy := P2.Y - P1.Y;

// Prevent division by zero and stop motion if the arc is undefined (points are collinear)
D := 2 * (Wx * (P3.Y - P2.Y) - Wy * (P3.X - P2.X));

IF D = 0 THEN
    SpeedX := 0;
    SpeedY := 0;
    RETURN;
END_IF;

CenterX := ((P3.Y - P2.Y)*(Ux*Wx + Uy*Wy) - (P3.X - P2.X)*(Vx*Wx + Vy*Wy)) / D;
CenterY := ((P3.X - P2.X)*(Ux*Wx + Uy*Wy) - (P3.Y - P2.Y)*(Vx*Wx + Vy*Wy)) / -D;

Radius := SQRT((P1.X - CenterX)*(P1.X - CenterX) + (P1.Y - CenterY)*(P1.Y - CenterY));
StartAngle := FU_atan2(P1.Y - CenterY, P1.X - CenterX);
EndAngle := FU_atan2(P3.Y - CenterY, P3.X - CenterX);

angleP2 := FU_atan2(P2.Y - CenterY, P2.X - CenterX);

// Determine clockwise or counterclockwise direction based on arc passing through P2
IF ((StartAngle < EndAngle) AND (angleP2 > StartAngle) AND (angleP2 < EndAngle)) OR
   ((StartAngle > EndAngle) AND NOT (angleP2 > EndAngle AND angleP2 < StartAngle)) THEN
    CW := FALSE;
ELSE
    CW := TRUE;
END_IF;

// Set direction multiplier: -1 for clockwise, +1 for counterclockwise
IF CW = TRUE THEN
	Dir := -1.0;
ELSE
	Dir := 1.0;
END_IF

// Adjust DeltaAngle for proper arc direction considering CW/CCW
DeltaAngle := EndAngle - StartAngle;
IF DeltaAngle < 0 AND NOT CW THEN
    DeltaAngle := DeltaAngle + 2 * 3.1415926;
ELSIF DeltaAngle > 0 AND CW THEN
    DeltaAngle := DeltaAngle - 2 * 3.1415926;
END_IF;
DeltaAngle := ABS(DeltaAngle);
ArcLength := Radius * DeltaAngle;
t_acc := MaxSpeed / Accel;
s_acc := 0.5 * Accel * t_acc * t_acc;
s_const := ArcLength - 2 * s_acc;

IF s_const < 0 THEN
    t_acc := SQRT(ArcLength / Accel);
    t_const := 0.0;
    t_total := 2 * t_acc;
ELSE
    t_const := s_const / MaxSpeed;
    t_total := 2 * t_acc + t_const;
END_IF;

// Acceleration phase
IF t < t_acc THEN
    v := Accel * t;
    s_t := 0.5 * Accel * t * t;

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
