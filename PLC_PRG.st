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

    // Absolute destination of the current segment, used for checking if the target is reached
    Target : Pos2D := (X:=0.0, Y:=0.0);
    t : REAL := 0.0;         // Time since current segment started
    cycle_time : REAL := 0.001; // Simulation step time
    MaxSpeed : REAL := 100.0;
    Accel : REAL := 15.0;
    // Gains for the PI controllers
    Kp : REAL := 2.0;
    Ki : REAL := 0.1;
    // Current velocity components used by the motion function blocks
    CurSpeedX : REAL := 0.0;
    CurSpeedY : REAL := 0.0;

    max_error : INT := 1; // Maximum error to consider the target reached
    continuity_error : INT := 15;
    closing_distnace : BOOL; // Flag to indicate if the current segment is being closed with a PI controller


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
    MoveFB : FB_LinealMove;
    ArcFB : FB_CircularMove;
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
    ReturnHome := FALSE;
    Go2Canvas := TRUE;
    Reset := FALSE;
END_IF

// Control movement execution
IF MotionActive THEN
    t := t + cycle_time;
    if not closing_distnace then // we use regular trapezoidal motion
        // First move from outside the Canvas to first canvas point
        IF  NOT ReturnHome and Go2Canvas THEN
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
                DecelerateAtEnd := Path[CurrentIndex].Break,
                SpeedX := CurSpeedX,
                SpeedY := CurSpeedY
            );

        // Moving throught the canvas path
        ELSIF NOT ReturnHome AND CurrentIndex <= PathLength THEN
            // Convert current path segment to global coordinates
            CanvasP1 := FU_CanvasOffset(Path[CurrentIndex].P1, CanvPos);
            CanvasP2 := FU_CanvasOffset(Path[CurrentIndex].P2, CanvPos);
            CanvasP3 := FU_CanvasOffset(Path[CurrentIndex].P3, CanvPos);
            Target := CanvasP3; // Set target to the end of the current segment
            IF NOT Path[CurrentIndex].IsArc THEN
                // Handle linear motion segments using PI control
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
            ELSE
                // Handle arc-based motion segments using PI control
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
            END_IF
        ELSIF ReturnHome THEN
            // Move back to the home position after completing the path
            MoveFB(
                StartPos := Path[PathLength].P3, // Use last point of the path as start
                TargetPos := HomePos,
                MaxSpeed := MaxSpeed,
                Accel := Accel,
                t := t,
                DecelerateAtEnd := Path[CurrentIndex].Break,
                SpeedX := CurSpeedX,
                SpeedY := CurSpeedY
            );
            Target := HomePos;
        END_IF
    ELSE // we're closing in to the point, using PI Control
        MovePI(
            CurPos := TCPAbs,
            TargetPos := Target,
            MaxSpeed := MaxSpeed,
            Ts := cycle_time,
            Kp := Kp,
            Ki := Ki,
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
    IF ABS(TCPAbs.X - Target.X) < continuity_error AND ABS(TCPAbs.Y - Target.Y) < continuity_error THEN
        IF Go2Canvas or Path[CurrentIndex].Break THEN
            // If the current segment is a break point or we are moving to canvas or returning home
            // we need to wait until we sto completely before moving to the next segment
            closing_distnace := TRUE; // Set flag to indicate we are closing distance

            IF ABS(TCPAbs.X - Target.X) < max_error AND ABS(TCPAbs.Y - Target.Y) < max_error THEN
                closing_distnace := FALSE; // Reset closing distance flag
                IF not Go2Canvas THEN
                    CurrentIndex := CurrentIndex + 1;
                END_IF;
                Go2Canvas := FALSE;
            END_IF
        ELSIF not Go2Canvas THEN // We are not at a break point, so we can move to the next segment without stoppingç
            IF ABS(TCPAbs.X - Target.X) < max_error AND ABS(TCPAbs.Y - Target.Y) < max_error THEN
                CurrentIndex := CurrentIndex + 1;
            END_IF;
        END_IF
		
        IF CurrentIndex > PathLength AND NOT ReturnHome THEN
            // Begin return to origin on completion
            ReturnHome := TRUE;
            ReturnStart := TCPRel;
        END_IF;
        IF ReturnHome AND ABS(TCPAbs.X - Target.X) < max_error AND ABS(TCPAbs.Y - Target.Y) < max_error THEN
            // Set target to home position for return
            Target := HomePos;
            MotionActive := FALSE;
            ReturnHome := FALSE;
        END_IF;
    END_IF;
END_IF;