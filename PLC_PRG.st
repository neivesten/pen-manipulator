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
    cycle_time : REAL := 0.01; // Simulation step time
    rMaxSpeed : REAL := 20.0;
    Accel : REAL := 15;
    // Gains for the PI controllers
    Kp : REAL := 2.0;
    Ki : REAL := 0.1;
    // Current velocity components used by the motion function blocks
    CurSpeedX : REAL := 0.0;
    CurSpeedY : REAL := 0.0;
	ModuleSpeed : REAL := 0.0; //module of Cur Speed X/Y

    max_error : INT := 1; // Maximum error to consider the target reached
    continuity_error : INT := 15;
    closing_distnace : BOOL; // Flag to indicate if the current segment is being closed with a PI controller


    MotionActive : BOOL := TRUE;
    Reset : BOOL := FALSE;

    
// Scaled Emoji coordinates for 250x250 image
Emoji : ARRAY[1..20] OF pos2D := [
    (X:=14.11, Y:=70.56),    // 1
    (X:=16.13, Y:=54.44),    // 2
    (X:=32.26, Y:=52.42),    // 3
    (X:=217.74, Y:=52.42),   // 4
    (X:=233.87, Y:=54.44),   // 5
    (X:=235.89, Y:=70.56),   // 6
    (X:=235.89, Y:=173.47),  // 7
    (X:=233.87, Y:=189.60),  // 8
    (X:=217.74, Y:=191.61),  // 9
    (X:=32.26, Y:=191.61),   // 10
    (X:=16.13, Y:=189.60),   // 11
    (X:=14.11, Y:=173.47),   // 12
    (X:=68.52, Y:=110.81),   // 13
    (X:=94.36, Y:=130.04),   // 14
    (X:=94.36, Y:=100.81),   // 15
    (X:=125.0, Y:=108.87),   // 16
    (X:=157.26, Y:=100.81),  // 17
    (X:=157.26, Y:=130.04),  // 18
    (X:=125.0, Y:=171.45),   // 19
    (X:=181.45, Y:=110.81)   // 20
];

 
	Path : ARRAY[1..17] OF PathPoint := [
        (P1:=Emoji[1], P2:=Emoji[2], P3:=Emoji[3], IsArc:=TRUE, Break:=FALSE, ZNottouching:= FALSE), //1
        (P1:=Emoji[3], P2:=Emoji[4], P3:=Emoji[4], IsArc:=FALSE, Break:=FALSE, ZNottouching:= FALSE), //2
		(P1:=Emoji[4], P2:=Emoji[5], P3:=Emoji[6], IsArc:=TRUE, Break:=FALSE, ZNottouching:= FALSE), //3
		(P1:=Emoji[6], P2:=Emoji[7], P3:=Emoji[7], IsArc:=FALSE, Break:=FALSE, ZNottouching:= FALSE), //4
		(P1:=Emoji[7], P2:=Emoji[8], P3:=Emoji[9], IsArc:=TRUE, Break:=FALSE, ZNottouching:= FALSE), //5
		(P1:=Emoji[9], P2:=Emoji[10], P3:=Emoji[10], IsArc:=FALSE, Break:=FALSE, ZNottouching:= FALSE),
		(P1:=Emoji[10], P2:=Emoji[11], P3:=Emoji[12], IsArc:=TRUE, Break:=FALSE, ZNottouching:= FALSE),
		(P1:=Emoji[12], P2:=Emoji[1], P3:=Emoji[1], IsArc:=FALSE, Break:=TRUE, ZNottouching:= FALSE),
		(P1:=Emoji[1], P2:=Emoji[2], P3:=Emoji[2], IsArc:=FALSE, Break:=FALSE, ZNottouching:= TRUE), //jump to 2 no tracing
		(P1:=Emoji[2], P2:=Emoji[13], P3:=Emoji[13], IsArc:=FALSE, Break:=TRUE, ZNottouching:= FALSE),
		(P1:=Emoji[13], P2:=Emoji[14], P3:=Emoji[14], IsArc:=FALSE, Break:=FALSE, ZNottouching:= TRUE), //jump to 14
		(P1:=Emoji[14], P2:=Emoji[19], P3:=Emoji[19], IsArc:=FALSE, Break:=TRUE, ZNottouching:= FALSE),
		(P1:=Emoji[19], P2:=Emoji[18], P3:=Emoji[18], IsArc:=FALSE, Break:=FALSE, ZNottouching:= FALSE),
		(P1:=Emoji[18], P2:=Emoji[17], P3:=Emoji[16], IsArc:=TRUE, Break:=TRUE, ZNottouching:= FALSE),
		(P1:=Emoji[16], P2:=Emoji[15], P3:=Emoji[14], IsArc:=TRUE, Break:=TRUE, ZNottouching:= FALSE),
		(P1:=Emoji[14], P2:=Emoji[20], P3:=Emoji[20], IsArc:=FALSE, Break:=FALSE, ZNottouching:= TRUE),// jump to 20
		(P1:=Emoji[20], P2:=Emoji[5], P3:=Emoji[5], IsArc:=FALSE, Break:=TRUE, ZNottouching:= FALSE)
    ];

    CurrentIndex : INT := 1;
    PathLength : INT := 17;
    
    MovePI : FB_LinealMovePI;
    MoveFB : FB_LinealMove;
    ArcFB : FB_CircularMove;
    tcpFB: FB_TCPaxes;

    rAngle: REAL;          // direction angle 
	rAngleX: REAL;
	rAngleY: REAL ;
	iAngleX: INT;
	iAngleY: INT;
	iArrow1pos2x: INT;
	iArrow1pos2y: INT;
	iArrow2pos2x: INT;
	iArrow2pos2y: INT;

    bZnotTouching: BOOL:=FALSE;  

    // Absolute position of the canvas origin in the global frame
    CanvPos : Pos2D := (X:=125.0, Y:=120.0);
    // Temporary storage for path points converted to global coordinates
    CanvasP1 : Pos2D;
    CanvasP2 : Pos2D;
    CanvasP3 : Pos2D;
    // Flag used when returning to the origin
    Go2Canvas : BOOL := FALSE;
    ReturnHome : BOOL := FALSE;
    ReturnStart : Pos2D;
    HomePos : Pos2D := (X:=25.0, Y:=100.0);
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
	bZnotTouching := TRUE;
    // Reset velocity
    CurSpeedX := 0.0;
    CurSpeedY := 0.0;
	ModuleSpeed := 0.0;
    CurrentIndex := 1;
    ReturnHome := FALSE;
    t := 0.0;
    Go2Canvas := TRUE;
    Reset := FALSE;

    // Reset TCP Axes
	iArrow1pos2x:= VisuPosX+iAngleX -50;
	iArrow1pos2y:= VisuPosY+iAngleY;
	iArrow2pos2x:= VisuPosX-iAngleY;
	iArrow2pos2y:= VisuPosY+iAngleX-50;
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
                MaxSpeed := rMaxSpeed,
                Accel := Accel,
                t := t,
                DecelerateAtEnd := TRUE,
                SpeedX := CurSpeedX,
                SpeedY := CurSpeedY
            );
			bZnotTouching:= TRUE;
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
                    MaxSpeed := rMaxSpeed,
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
                    MaxSpeed := rMaxSpeed,
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
                StartPos := ReturnStart, // Use last point of the path as start
                TargetPos := HomePos,
                MaxSpeed := rMaxSpeed,
                Accel := Accel,
                t := t,
                DecelerateAtEnd := Path[CurrentIndex].Break,
                SpeedX := CurSpeedX,
                SpeedY := CurSpeedY
            );
            Target := HomePos;
			bZnotTouching:= TRUE;
        END_IF
    ELSE // we're closing in to the point, using PI Control
        MovePI(
            CurPos := TCPAbs,
            TargetPos := Target,
            MaxSpeed := rMaxSpeed,
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

    tcpFB(
		 SpeedX := CurSpeedX,
         SpeedY := CurSpeedY
		);
	
	rAngle:= tcpFB.rAngle;
	rAngle:= rAngle*180/3.1415;
	rAngleX:= tcpFB.rAngleX;
	rAngleY:= tcpFB.rAngleY;
	iAngleX:= TO_INT(rAngleX);
	iAngleY:= TO_INT(rAngleY);


    // Convert precise position to integer for visualization
    VisuPosX := TO_INT(TCPRel.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPRel.Y - CircleWidth / 2.0);
	
    // pos arrows tcp
	iArrow1pos2x:= VisuPosX+iAngleX -50;
	iArrow1pos2y:= VisuPosY+iAngleY;
	iArrow2pos2x:= VisuPosX-iAngleY;
	iArrow2pos2y:= VisuPosY+iAngleX-50;

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
                t := 0.0; // Reset time for the next segment
            END_IF
        ELSIF not Go2Canvas THEN // We are not at a break point, so we can move to the next segment without stopping
            IF ABS(TCPAbs.X - Target.X) < max_error AND ABS(TCPAbs.Y - Target.Y) < max_error THEN
                CurrentIndex := CurrentIndex + 1;
                t := 0.0; // Reset time for the next segment
				
            END_IF;
            
        END_IF
		
        IF CurrentIndex > PathLength AND NOT ReturnHome THEN
            // Begin return to origin on completion
            ReturnHome := TRUE;
            ReturnStart := TCPAbs;
			CurrentIndex := 1;
        ELSIF ReturnHome AND ABS(TCPAbs.X - Target.X) < max_error AND ABS(TCPAbs.Y - Target.Y) < max_error THEN
            // Set target to home position for return
            Target := HomePos;
            MotionActive := FALSE;
            ReturnHome := FALSE;
			CurSpeedX:= 0;
			CurSpeedY:= 0;
        END_IF;
    END_IF;
	
	ModuleSpeed:= SQRT(CurSpeedX*CurSpeedX+CurSpeedY*CurSpeedY);
	IF NOT (ReturnHome OR Go2Canvas) THEN
		bZnotTouching:= path[CurrentIndex].ZNottouching;
	END_IF
	
	
END_IF;
