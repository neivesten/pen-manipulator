PROGRAM PLC_PRG
VAR
    // Local TCP position measured from the first path point
    TCPRel : Pos2D := (X:=0.0, Y:=0.0);
    // Absolute TCP position used for distance checks
    TCPAbs : Pos2D := (X:=0.0, Y:=0.0);
    // Integer coordinates for the visualization
    VisuPosX : INT := 0;                // X position in pixels
    VisuPosY : INT := 0;                // Y position in pixels
    // Diameter of the circle used to draw the TCP
    CircleWidth : REAL := 10.0;
    // Flag used to trigger reset on startup
    InitDone : BOOL := FALSE;

    // Absolute destination of the current segment
    Target : Pos2D := (X:=0.0, Y:=0.0);

    cycle_time : REAL := 0.01; // Simulation step time
    MaxSpeed : REAL := 25.0;
    // Gains for the PI controllers
    Kp : REAL := 2.0;
    Ki : REAL := 1.0;
    // Current velocity components used by the motion function blocks
    CurSpeedX : REAL := 0.0;
    CurSpeedY : REAL := 0.0;


    MotionActive : BOOL := TRUE;
    Reset : BOOL := FALSE;

    Path : ARRAY[1..3] OF PathPoint := [
        (P1:=(X:=139.0, Y:=82.0), P2:=(X:=239.0, Y:=82.0), P3:=(X:=239.0, Y:=82.0), IsArc:=FALSE, Break:=FALSE),
        (P1:=(X:=239.0, Y:=82.0), P2:=(X:=364.0, Y:=182.0), P3:=(X:=239.0, Y:=282.0), IsArc:=TRUE, Break:=TRUE),
		(P1:=(X:=239.0, Y:=282.0), P2:=(X:=139.0, Y:=82.0), P3:=(X:=139.0, Y:=82.0), IsArc:=FALSE, Break:=TRUE)
    ];
    CurrentIndex : INT := 1;
    PathLength : INT := 3;
    MovePI : FB_LinealMovePI;
    ArcPI : FB_CircularMovePI;
    // Absolute position of the canvas origin in the global frame
    CanvPos : Pos2D := (X:=226.0, Y:=132.0);
    // Temporary storage for path points converted to global coordinates
    CanvasP1 : Pos2D;
    CanvasP2 : Pos2D;
    CanvasP3 : Pos2D;
    // Flag used when returning to the origin
    Go2Canvas : BOOL := FALSE;
    ReturnHome : BOOL := FALSE;
    ReturnStart : Pos2D;
    HomePos : Pos2D := (X:=25.0, Y:=150.0);
END_VAR

// Automatic reset on first cycle
IF NOT InitDone THEN
    Reset := TRUE;
    InitDone := TRUE;
END_IF;

// Reset position and initialise offsets
IF Reset THEN
    // TCP starts at the home position in absolute coordinates
    TCPAbs := HomePos;
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
    Go2Canvas := TRUE;
    Reset := FALSE;

    
END_IF

// Control movement execution
IF MotionActive THEN
    // First move from outside the Canvas to first canvas point
    IF  NOT ReturnHome and Go2Canvas THEN
        t := t + cycle_time;
        
        // Convert first path point from canvas to global coordinates
        CanvasP2 := FU_CanvasOffset(Path[1].P1, CanvPos);
        Target := CanvasP2;
        // Move to initial position in canvas
        MoveFB(
                StartPos := HomePos,
                TargetPos := CanvasP2,
                MaxSpeed := MaxSpeed,
                Accel := Accel,
                t := t,
                DecelerateAtEnd := TRUE,
                SpeedX := CurSpeedX,
                SpeedY := CurSpeedY
            );

    // Moving throught the canvas path
    ELSIF NOT ReturnHome AND CurrentIndex <= PathLength THEN
        t := t + cycle_time;

        // Convert current path segment to global coordinates
        CanvasP1 := FU_CanvasOffset(Path[CurrentIndex].P1, CanvPos);
        CanvasP2 := FU_CanvasOffset(Path[CurrentIndex].P2, CanvPos);
        CanvasP3 := FU_CanvasOffset(Path[CurrentIndex].P3, CanvPos);

    IF NOT Path[CurrentIndex].IsArc THEN
        // Handle linear motion segments using PI control
        MovePI(
            CurPos := TCPAbs,
            TargetPos := CanvasP2,
            MaxSpeed := MaxSpeed,
            Kp := Kp,
            Ki := Ki,
            Ts := cycle_time,
            SpeedX := CurSpeedX,
            SpeedY := CurSpeedY
        );

        // Integrate velocity to update position
        TCPAbs.X := TCPAbs.X + CurSpeedX * cycle_time;
        TCPAbs.Y := TCPAbs.Y + CurSpeedY * cycle_time;
        Target := Path[CurrentIndex].P2;
    ELSE
        // Handle arc-based motion segments using PI control
        ArcPI(
            CurPos := TCPAbs,
            P1 := Path[CurrentIndex].P1,
            P2 := Path[CurrentIndex].P2,
            P3 := Path[CurrentIndex].P3,
            MaxSpeed := MaxSpeed,
            Kp := Kp,
            Ki := Ki,
            Ts := cycle_time,
            SpeedX := CurSpeedX,
            SpeedY := CurSpeedY
        );
    END_IF

    // Integrate velocity to update position
    TCPAbs.X := TCPAbs.X + CurSpeedX * cycle_time;
    TCPAbs.Y := TCPAbs.Y + CurSpeedY * cycle_time;
    // Update relative coordinates after integrating
    TCPRel.X := TCPAbs.X - HomePos.X;
    TCPRel.Y := TCPAbs.Y - HomePos.Y;

    // Convert precise position to integer for visualization
    VisuPosX := TO_INT(TCPRel.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPRel.Y - CircleWidth / 2.0);

    // Check if current target is reached and progress to next segment
    IF ABS(TCPAbs.X - Target.X) < 1 AND ABS(TCPAbs.Y - Target.Y) < 1 THEN
        CurrentIndex := CurrentIndex + 1;
		Go2Canvas := FALSE;
        IF CurrentIndex > PathLength THEN
            // Begin return to origin on completion
            ReturnHome := TRUE;
            ReturnStart := TCPRel;
            t := 0.0;
        END_IF;
    END_IF;

    IF ReturnHome AND ABS(TCPRel.X) < 1 AND ABS(TCPRel.Y) < 1 THEN
        MotionActive := FALSE;
        ReturnHome := FALSE;
    END_IF;
END_IF;

// End of PLC_PRG
