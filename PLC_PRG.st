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
	
	//Emoji coordinates in the Emoji visu coordinate system
	Emoji :  ARRAY[1..20] OF pos2D := [
        (X:=45.0, Y:=175.0), // 1
 		(X:=50.0, Y:=135.0), // 2
		(X:=90.0, Y:=130.0), // 3
		(X:=550.0, Y:=130.0), // 4
		(X:=590.0, Y:=135.0), // 5
		(X:=595.0, Y:=175.0), // 6
		(X:=595.0, Y:=430.0), // 7
		(X:=590.0, Y:=470.0), // 8
		(X:=550.0, Y:=475.0), // 9
		(X:=90.0, Y:=475.0), // 10
		(X:=50.0, Y:=470.0), // 11
 		(X:=45.0, Y:=430.0), // 12
		(X:=180.0, Y:=275.0), // 13
		(X:=245.0, Y:=325.0), // 14
		(X:=245.0, Y:=250.0), // 15
		(X:=320.0, Y:=270.0), // 16
		(X:=400.0, Y:=250.0), // 17
		(X:=400.0, Y:=325.0), // 18
		(X:=320.0, Y:=425.0), // 19
		(X:=460.0, Y:=275.0) // 20
    ];
	EmojiPath : ARRAY[1..3] OF PathPoint := [
        (P1:=Emoji[1], P2:=Emoji[2], P3:=Emoji[3], IsArc:=TRUE, Break:=FALSE),
        (P1:=Emoji[3], P2:=Emoji[4], P3:=Emoji[4], IsArc:=FALSE, Break:=FALSE),
		(P1:=Emoji[4], P2:=Emoji[5], P3:=Emoji[6], IsArc:=TRUE, Break:=FALSE),
		(P1:=Emoji[6], P2:=Emoji[7], P3:=Emoji[7], IsArc:=FALSE, Break:=FALSE),
		(P1:=Emoji[7], P2:=Emoji[8], P3:=Emoji[9], IsArc:=TRUE, Break:=FALSE),
		(P1:=Emoji[9], P2:=Emoji[10], P3:=Emoji[10], IsArc:=FALSE, Break:=FALSE),
		(P1:=Emoji[10], P2:=Emoji[11], P3:=Emoji[12], IsArc:=TRUE, Break:=FALSE),
		(P1:=Emoji[12], P2:=Emoji[1], P3:=Emoji[1], IsArc:=FALSE, Break:=TRUE),
		(P1:=Emoji[1], P2:=Emoji[2], P3:=Emoji[2], IsArc:=FALSE, Break:=FALSE), //jump to 2 no tracing
		(P1:=Emoji[2], P2:=Emoji[13], P3:=Emoji[13], IsArc:=FALSE, Break:=TRUE),
		(P1:=Emoji[13], P2:=Emoji[14], P3:=Emoji[14], IsArc:=FALSE, Break:=FALSE), //jump to 14
		(P1:=Emoji[14], P2:=Emoji[15], P3:=Emoji[16], IsArc:=TRUE, Break:=FALSE),
		(P1:=Emoji[16], P2:=Emoji[17], P3:=Emoji[18], IsArc:=TRUE, Break:=FALSE),
		(P1:=Emoji[18], P2:=Emoji[19], P3:=Emoji[19], IsArc:=FALSE, Break:=FALSE),
		(P1:=Emoji[19], P2:=Emoji[14], P3:=Emoji[14], IsArc:=FALSE, Break:=TRUE),
		(P1:=Emoji[14], P2:=Emoji[20], P3:=Emoji[20], IsArc:=FALSE, Break:=FALSE),// jump to 20
		(P1:=Emoji[20], P2:=Emoji[5], P3:=Emoji[5], IsArc:=FALSE, Break:=TRUE),
    ];
    CurrentIndex : INT := 1;
    PathLength : INT := 3;
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

    IF NOT Path[CurrentIndex].IsArc THEN
        // Handle linear motion segments
        MoveFB(
            StartPos := Path[CurrentIndex].P1,
            TargetPos := Path[CurrentIndex].P2,
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
            P1 := Path[CurrentIndex].P1,
            P2 := Path[CurrentIndex].P2,
            P3 := Path[CurrentIndex].P3,
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
	
	
    // Update local coordinate from absolute position
    TCPRel.X := TCPAbs.X - StartOffset.X;
    TCPRel.Y := TCPAbs.Y - StartOffset.Y;

    // Convert precise position to integer for visualization
    VisuPosX := TO_INT(TCPRel.X - CircleWidth / 2.0);
    VisuPosY := TO_INT(TCPRel.Y - CircleWidth / 2.0);

	// pos arrows tcp
	iArrow1pos2x:= VisuPosX+iAngleX;
	iArrow1pos2y:= VisuPosY+iAngleY;
	iArrow2pos2x:= VisuPosX-iAngleX;
	iArrow2pos2y:= VisuPosX+iAngleY;
	
    // Check if current target is reached and progress to next segment
    IF ABS(TCPAbs.X - Target.X) < 1 AND ABS(TCPAbs.Y - Target.Y) < 1 THEN
        t := 0.0;
        CurrentIndex := CurrentIndex + 1;
    END_IF;
END_IF;

// End of PLC_PRG

