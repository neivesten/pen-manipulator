//
// Main program that iterates through an array of PathPoint records and
// feeds the motion function blocks.  The current position is integrated
// from the velocity outputs of the blocks to provide a simple simulation
// of the manipulator motion.
//
PROGRAM PLC_PRG
VAR
    // Visualization and TCP tracking variables
    TCPPos : Pos2D := (X:=0.0, Y:=0.0); // TCP position in local coordinates
    VisuPosX : INT := 0;                // Visualization X (upper left origin)
    VisuPosY : INT := 0;                // Visualization Y (upper left origin)
    StartOffset : Pos2D := (X:=0.0, Y:=0.0); // Offset of first path point
    CircleWidth : REAL := 10.0;         // Visual element width for centring
    InitDone : BOOL := FALSE;

    TargetX : REAL := 0.0;
    TargetY : REAL := 0.0;

    t : REAL := 0.0;         // Time since current segment started
    cycle_time : REAL := 0.1; // Simulation step time

    MaxSpeed : REAL := 50.0;
    Accel : REAL := 100.0;

    MotionActive : BOOL := TRUE;
    Reset : BOOL := FALSE;
    v : REAL := 0.0;

    Path : ARRAY[1..2] OF PathPoint := [
        (P1:=(X:=0.0, Y:=0.0), P2:=(X:=100.0, Y:=0.0), P3:=(X:=100.0, Y:=0.0), IsArc:=FALSE, End:=FALSE),
        (P1:=(X:=100.0, Y:=0.0), P2:=(X:=150.0, Y:=50.0), P3:=(X:=100.0, Y:=100.0), IsArc:=TRUE, End:=TRUE)
    ];
    CurrentIndex : INT := 1;
    PathLength : INT := 2;
    MoveFB : FB_TrapezoidalMove;
    ArcFB : FB_CircularMove;
    StartRel : Pos2D;
    MidRel : Pos2D;
    EndRel : Pos2D;
END_VAR

// Automatic reset on first cycle
IF NOT InitDone THEN
    Reset := TRUE;
    InitDone := TRUE;
END_IF;

// Reset position and speed values
IF Reset THEN
    StartOffset.X := Path[1].P1.X;
    StartOffset.Y := Path[1].P1.Y;

    TCPPos.X := 0.0;
    TCPPos.Y := 0.0;

    VisuPosX := TO_INT(TCPPos.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPPos.Y - CircleWidth / 2.0);

    v := 0; // Reset velocity
    Reset := FALSE;
END_IF

// Control movement execution
IF MotionActive AND CurrentIndex <= PathLength THEN
    t := t + cycle_time;

    StartRel.X := Path[CurrentIndex].P1.X - StartOffset.X;
    StartRel.Y := Path[CurrentIndex].P1.Y - StartOffset.Y;

    IF NOT Path[CurrentIndex].IsArc THEN
        EndRel.X := Path[CurrentIndex].P2.X - StartOffset.X;
        EndRel.Y := Path[CurrentIndex].P2.Y - StartOffset.Y;

        // Handle linear motion segments
        MoveFB(
            StartPos := StartRel,
            TargetPos := EndRel,
            MaxSpeed := MaxSpeed,
            Accel := Accel,
            t := t,
            DecelerateAtEnd := Path[CurrentIndex].End
        );

        // Integrate velocity to update position
        TCPPos.X := TCPPos.X + MoveFB.SpeedX * cycle_time;
        TCPPos.Y := TCPPos.Y + MoveFB.SpeedY * cycle_time;
        TargetX := EndRel.X;
        TargetY := EndRel.Y;
    ELSE
        MidRel.X := Path[CurrentIndex].P2.X - StartOffset.X;
        MidRel.Y := Path[CurrentIndex].P2.Y - StartOffset.Y;
        EndRel.X := Path[CurrentIndex].P3.X - StartOffset.X;
        EndRel.Y := Path[CurrentIndex].P3.Y - StartOffset.Y;

        // Handle arc-based motion segments
        ArcFB(
            P1 := StartRel,
            P2 := MidRel,
            P3 := EndRel,
            MaxSpeed := MaxSpeed,
            Accel := Accel,
            t := t,
            DecelerateAtEnd := Path[CurrentIndex].End
        );

        // Integrate velocity to update position
        TCPPos.X := TCPPos.X + ArcFB.SpeedX * cycle_time;
        TCPPos.Y := TCPPos.Y + ArcFB.SpeedY * cycle_time;
        TargetX := EndRel.X;
        TargetY := EndRel.Y;
    END_IF;

    // Convert precise position to integer for visualization
    VisuPosX := TO_INT(TCPPos.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPPos.Y - CircleWidth / 2.0);

    // Check if current target is reached and progress to next segment
    IF ABS(TCPPos.X - TargetX) < 0.5 AND ABS(TCPPos.Y - TargetY) < 0.5 THEN
        t := 0.0;
        CurrentIndex := CurrentIndex + 1;

        IF Path[CurrentIndex - 1].End THEN
            MotionActive := FALSE;
        END_IF;
    END_IF;
END_IF;

// End of PLC_PRG
