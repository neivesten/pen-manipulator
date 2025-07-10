PROGRAM PLC_PRG
VAR
    // Local TCP position measured from the first path point
    TCPRel : Pos2D := (X:=0.0, Y:=0.0);
    // Absolute TCP position used for distance checks
    TCPAbs : Pos2D := (X:=0.0, Y:=0.0);
    // Integer coordinates for the visualization
    VisuPosX : INT := 0;                // X position in pixels
    VisuPosY : INT := 0;                // Y position in pixels
    // Absolute offset of the canvas origin
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
    // Helper to translate coordinates into the canvas frame
    CanvasTrans : FU_CanvasOffset;
    // Offset used for translating the path
    PathOffset : Pos2D := (X:=0.0, Y:=0.0);
    // Temporary positions after translation
    TransP1, TransP2, TransP3 : Pos2D;
END_VAR

// Automatic reset on first cycle
IF NOT InitDone THEN
    Reset := TRUE;
    InitDone := TRUE;
END_IF;

// Reset position and initialise offsets
IF Reset THEN
    // Offset of the path relative to the canvas
    PathOffset := Path[1].P1;

    // Start at the canvas origin (0,0)
    StartOffset := (X:=0.0, Y:=0.0);

    // TCP begins at the origin in absolute coordinates
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

    IF NOT Path[CurrentIndex].IsArc THEN
        // Handle linear motion segments
        // Translate path coordinates to the canvas frame
        CanvasTrans(Source := Path[CurrentIndex].P1, Offset := PathOffset);
        TransP1 := CanvasTrans.Canvas;
        CanvasTrans(Source := Path[CurrentIndex].P2, Offset := PathOffset);
        TransP2 := CanvasTrans.Canvas;

        MoveFB(
            StartPos := TransP1,
            TargetPos := TransP2,
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
        Target := TransP2;
    ELSE
        // Handle arc-based motion segments
        // Translate arc points to the canvas frame
        CanvasTrans(Source := Path[CurrentIndex].P1, Offset := PathOffset);
        TransP1 := CanvasTrans.Canvas;
        CanvasTrans(Source := Path[CurrentIndex].P2, Offset := PathOffset);
        TransP2 := CanvasTrans.Canvas;
        CanvasTrans(Source := Path[CurrentIndex].P3, Offset := PathOffset);
        TransP3 := CanvasTrans.Canvas;

        ArcFB(
            P1 := TransP1,
            P2 := TransP2,
            P3 := TransP3,
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
        Target := TransP3;
    END_IF;

    // Update local coordinate from absolute position
    TCPRel.X := TCPAbs.X - StartOffset.X;
    TCPRel.Y := TCPAbs.Y - StartOffset.Y;

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
