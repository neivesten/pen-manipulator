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
    // Utility block to translate positions into the canvas frame
    OffsetFB : FB_CanvasOffset;
    // Temporary storage for path points in canvas coordinates
    CanvasP1 : Pos2D;
    CanvasP2 : Pos2D;
    CanvasP3 : Pos2D;
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
    Reset := FALSE;

    
END_IF

// Control movement execution
IF MotionActive AND CurrentIndex <= PathLength THEN
    t := t + cycle_time;

    // Translate current segment points into canvas coordinates
    OffsetFB(Source := Path[CurrentIndex].P1, Offset := StartOffset);
    CanvasP1 := OffsetFB.Canvas;
    OffsetFB(Source := Path[CurrentIndex].P2, Offset := StartOffset);
    CanvasP2 := OffsetFB.Canvas;
    OffsetFB(Source := Path[CurrentIndex].P3, Offset := StartOffset);
    CanvasP3 := OffsetFB.Canvas;

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

    // Update local coordinate from absolute position using FB_CanvasOffset
    OffsetFB(Source := TCPAbs, Offset := StartOffset);
    TCPRel := OffsetFB.Canvas;

    // Convert precise position to integer for visualization
    VisuPosX := TO_INT(TCPRel.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPRel.Y - CircleWidth / 2.0);

    // Check if current target is reached and progress to next segment
    IF ABS(TCPAbs.X - Target.X) < 1 AND ABS(TCPAbs.Y - Target.Y) < 1 THEN
        t := 0.0;
        CurrentIndex := CurrentIndex + 1;
    END_IF;
END_IF;

// End of PLC_PRG
