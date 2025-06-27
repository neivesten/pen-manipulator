PROGRAM PLC_PRG
VAR
    // Visualization and motion tracking variables
    VisuX_tmp : REAL := 0.0; // Internal X position
    VisuY_tmp : REAL := 0.0; // Internal Y position
    VisuPosX : INT := 0;     // Position used in visualization
    VisuPosY : INT := 0;

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
END_VAR

// Reset position and speed values
IF Reset THEN
	
	VisuPosX := 0; // Reset visualization X position
    VisuPosY := 0; // Reset visualization Y position
	
	VisuX_tmp := 0.0; // Reset precise X position
    VisuY_tmp := 0.0; // Reset precise Y position
	
	v := 0; // Reset velocity
	
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
        VisuX_tmp := VisuX_tmp + MoveFB.SpeedX * cycle_time;
        VisuY_tmp := VisuY_tmp + MoveFB.SpeedY * cycle_time;
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
        VisuX_tmp := VisuX_tmp + ArcFB.SpeedX * cycle_time;
        VisuY_tmp := VisuY_tmp + ArcFB.SpeedY * cycle_time;
    END_IF;

    // Convert precise position to integer for visualization
    VisuPosX := TO_INT(VisuX_tmp);
    VisuPosY := TO_INT(VisuY_tmp);

    // Set destination for current segment
    TargetX := Path[CurrentIndex].P3.X;
    TargetY := Path[CurrentIndex].P3.Y;

    // Check if current target is reached and progress to next segment
    IF ABS(VisuX_tmp - TargetX) < 0.5 AND ABS(VisuY_tmp - TargetY) < 0.5 THEN
        t := 0.0;
        CurrentIndex := CurrentIndex + 1;

        IF Path[CurrentIndex - 1].End THEN
            MotionActive := FALSE;
        END_IF;
    END_IF;
END_IF;