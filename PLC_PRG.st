//
// Main program that iterates through an array of PathPoint records and
// feeds the motion function blocks.  The current position is integrated
// from the velocity outputs of the blocks to provide a simple simulation
// of the manipulator motion.
//
PROGRAM PLC_PRG
VAR
    // Local TCP position measured from the first path point
    TCPRel : Pos2D := (X:=0.0, Y:=0.0);
    // Absolute TCP position used for distance checks
    TCPAbs : Pos2D := (X:=0.0, Y:=0.0);
    // Integer coordinates for the visualization
    VisuPosX : INT := 0;                // X position in pixels
    VisuPosY : INT := 0;                // Y position in pixels
    // Absolute position of the first path point
    StartOffset : Pos2D := (X:=0.0, Y:=0.0);
    // Diameter of the circle used to draw the TCP
    CircleWidth : REAL := 10.0;
    // Flag used to trigger reset on startup
    InitDone : BOOL := FALSE;

    // Absolute destination of the current segment
    Target : Pos2D := (X:=0.0, Y:=0.0);

    t : REAL := 0.0;         // Time since current segment started
    cycle_time : REAL := 0.1; // Simulation step time

    MaxSpeed : REAL := 50.0;
    Accel : REAL := 100.0;

    MotionActive : BOOL := TRUE;
    Reset : BOOL := FALSE;

    Path : ARRAY[1..2] OF PathPoint := [
        (P1:=(X:=0.0, Y:=0.0), P2:=(X:=100.0, Y:=0.0), P3:=(X:=100.0, Y:=0.0), IsArc:=FALSE, End:=FALSE),
        (P1:=(X:=100.0, Y:=0.0), P2:=(X:=150.0, Y:=50.0), P3:=(X:=100.0, Y:=100.0), IsArc:=TRUE, End:=TRUE)
    ];
    CurrentIndex : INT := 1;
    PathLength : INT := 2;
    MoveFB : FB_TrapezoidalMove;
    ArcFB : FB_CircularMove;
END_VAR

// Automatic reset on first cycle
IF NOT InitDone THEN
    Reset := TRUE;
    InitDone := TRUE;
END_IF;

// Reset position and initialise offsets
IF Reset THEN
    // Record the absolute starting position
    StartOffset := Path[1].P1;

    // TCP starts at the first path point in absolute coordinates
    TCPAbs := StartOffset;
    // No displacement in the local frame at reset
    TCPRel.X := 0.0;
    TCPRel.Y := 0.0;

    // Initialise the visualisation values
    VisuPosX := TO_INT(TCPRel.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPRel.Y - CircleWidth / 2.0);

    Reset := FALSE;
END_IF

// Control movement execution
IF MotionActive AND CurrentIndex <= PathLength THEN
    t := t + cycle_time;

    IF NOT Path[CurrentIndex].IsArc THEN
        // Handle linear motion segments
        MoveFB(
            StartPos := Path[CurrentIndex].P1,
            TargetPos := Path[CurrentIndex].P2,
            MaxSpeed := MaxSpeed,
            Accel := Accel,
            t := t,
            DecelerateAtEnd := Path[CurrentIndex].End
        );

        // Integrate velocity to update position
        TCPAbs.X := TCPAbs.X + MoveFB.SpeedX * cycle_time;
        TCPAbs.Y := TCPAbs.Y + MoveFB.SpeedY * cycle_time;
        Target := Path[CurrentIndex].P2;
    ELSE
        // Handle arc-based motion segments
        ArcFB(
            P1 := Path[CurrentIndex].P1,
            P2 := Path[CurrentIndex].P2,
            P3 := Path[CurrentIndex].P3,
            MaxSpeed := MaxSpeed,
            Accel := Accel,
            t := t,
            DecelerateAtEnd := Path[CurrentIndex].End
        );

        // Integrate velocity to update position
        TCPAbs.X := TCPAbs.X + ArcFB.SpeedX * cycle_time;
        TCPAbs.Y := TCPAbs.Y + ArcFB.SpeedY * cycle_time;
        Target := Path[CurrentIndex].P3;
    END_IF;

    // Update local coordinate from absolute position
    TCPRel.X := TCPAbs.X - StartOffset.X;
    TCPRel.Y := TCPAbs.Y - StartOffset.Y;

    // Convert precise position to integer for visualization
    VisuPosX := TO_INT(TCPRel.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPRel.Y - CircleWidth / 2.0);

    // Check if current target is reached and progress to next segment
    IF ABS(TCPAbs.X - Target.X) < 0.5 AND ABS(TCPAbs.Y - Target.Y) < 0.5 THEN
        t := 0.0;
        CurrentIndex := CurrentIndex + 1;

        IF Path[CurrentIndex - 1].End THEN
            MotionActive := FALSE;
        END_IF;
    END_IF;
END_IF;

// End of PLC_PRG
