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
    cycle_time : REAL := 0.01; // Simulation step time
    MaxSpeed : REAL := 25.0;
    Accel : REAL := 15.0;
    // Current velocity components used by the motion function blocks
    CurSpeedX : REAL := 0.0;
    CurSpeedY : REAL := 0.0;


    MotionActive : BOOL := TRUE;
    Reset : BOOL := FALSE;

    Path : ARRAY[1..3] OF PathPoint := [
        (P1:=(X:=25.0, Y:=150.0), P2:=(X:=125.0, Y:=150.0), P3:=(X:=125.0, Y:=150.0), IsArc:=FALSE, Break:=FALSE),
        (P1:=(X:=125.0, Y:=150.0), P2:=(X:=250.0, Y:=250.0), P3:=(X:=125.0, Y:=350.0), IsArc:=TRUE, Break:=TRUE),
		(P1:=(X:=125.0, Y:=350.0), P2:=(X:=25.0, Y:=150.0), P3:=(X:=25.0, Y:=150.0), IsArc:=FALSE, Break:=TRUE)
    ];
    CurrentIndex : INT := 1;
    PathLength : INT := 3;
    MoveFB : FB_LinealMove;
    ArcFB : FB_CircularMove;
    // Pre-computed path expressed in canvas coordinates
    CanvasPath : ARRAY[1..3] OF PathPoint;
    // Temporary storage for path points in canvas coordinates
    CanvasP1 : Pos2D;
    CanvasP2 : Pos2D;
    CanvasP3 : Pos2D;
    // Flag used when returning to the origin
    ReturnHome : BOOL := FALSE;
    ReturnStart : Pos2D;
    i : INT;
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
    // Reset velocity
    CurSpeedX := 0.0;
    CurSpeedY := 0.0;
    CurrentIndex := 1;
    t := 0.0;
    ReturnHome := FALSE;

    // Pre-compute path points in canvas coordinates
    FOR i := 1 TO PathLength DO
        CanvasPath[i].P1 := FU_CanvasOffset(Path[i].P1, StartOffset);
        CanvasPath[i].P2 := FU_CanvasOffset(Path[i].P2, StartOffset);
        CanvasPath[i].P3 := FU_CanvasOffset(Path[i].P3, StartOffset);
        CanvasPath[i].IsArc := Path[i].IsArc;
        CanvasPath[i].Break := Path[i].Break;
    END_FOR;
    Reset := FALSE;

    
END_IF

// Control movement execution
IF MotionActive AND NOT ReturnHome AND CurrentIndex <= PathLength THEN
    t := t + cycle_time;

    // Fetch pre-translated path points in canvas coordinates
    CanvasP1 := CanvasPath[CurrentIndex].P1;
    CanvasP2 := CanvasPath[CurrentIndex].P2;
    CanvasP3 := CanvasPath[CurrentIndex].P3;

    IF NOT Path[CurrentIndex].IsArc THEN
        // Handle linear motion segments
        MoveFB(
            StartPos := CanvasP1,
            TargetPos := CanvasP2,
            MaxSpeed := MaxSpeed,
            Accel := Accel,
            t := t,
            DecelerateAtEnd := Path[CurrentIndex].Break,
            SpeedX := CurSpeedX,
            SpeedY := CurSpeedY
        );

        // Integrate velocity to update position
        TCPAbs.X := TCPAbs.X + CurSpeedX * cycle_time;
        TCPAbs.Y := TCPAbs.Y + CurSpeedY * cycle_time;
        Target := Path[CurrentIndex].P2;
    ELSE
        // Handle arc-based motion segments
        ArcFB(
            P1 := CanvasP1,
            P2 := CanvasP2,
            P3 := CanvasP3,
            MaxSpeed := MaxSpeed,
            Accel := Accel,
            t := t,
            DecelerateAtEnd := Path[CurrentIndex].Break,
            SpeedX := CurSpeedX,
            SpeedY := CurSpeedY
        );

        // Integrate velocity to update position
        TCPAbs.X := TCPAbs.X + CurSpeedX * cycle_time;
        TCPAbs.Y := TCPAbs.Y + CurSpeedY * cycle_time;
        Target := Path[CurrentIndex].P3;
    END_IF;

    // Update local coordinate from absolute position
    TCPRel := FU_CanvasOffset(TCPAbs, StartOffset);

    // Convert precise position to integer for visualization
    VisuPosX := TO_INT(TCPRel.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPRel.Y - CircleWidth / 2.0);

    // Check if current target is reached and progress to next segment
    IF ABS(TCPAbs.X - Target.X) < 1 AND ABS(TCPAbs.Y - Target.Y) < 1 THEN
        t := 0.0;
        CurrentIndex := CurrentIndex + 1;
        IF CurrentIndex > PathLength THEN
            // Begin return to origin on completion
            ReturnHome := TRUE;
            ReturnStart := TCPRel;
            t := 0.0;
        END_IF;
    END_IF;
ELSIF MotionActive AND ReturnHome THEN
    t := t + cycle_time;
    MoveFB(
        StartPos := ReturnStart,
        TargetPos := (X:=0.0, Y:=0.0),
        MaxSpeed := MaxSpeed,
        Accel := Accel,
        t := t,
        DecelerateAtEnd := TRUE,
        SpeedX := CurSpeedX,
        SpeedY := CurSpeedY
    );

    TCPAbs.X := TCPAbs.X + CurSpeedX * cycle_time;
    TCPAbs.Y := TCPAbs.Y + CurSpeedY * cycle_time;

    TCPRel := FU_CanvasOffset(TCPAbs, StartOffset);

    VisuPosX := TO_INT(TCPRel.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPRel.Y - CircleWidth / 2.0);

    IF ABS(TCPRel.X) < 1 AND ABS(TCPRel.Y) < 1 THEN
        MotionActive := FALSE;
        ReturnHome := FALSE;
    END_IF;
END_IF;

// End of PLC_PRG
