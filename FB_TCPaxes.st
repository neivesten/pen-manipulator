//
// Function block implementing a trapezoidal speed profile for linear
// movements.  The block accelerates from the start position, maintains a
// constant speed and optionally decelerates at the end.
//
FUNCTION_BLOCK FB_TCPaxes
VAR_INPUT
	SpeedX : REAL;      // Current speed in the X direction (initial value on first call)
    SpeedY : REAL;      // Current speed in the Y direction (initial value on first call)
END_VAR
VAR_OUTPUT
    rAngle: REAL;          // direction angle 
	rAngleX: REAL;
	rAngleY: REAL;
END_VAR

	rAngle := FU_atan2(SpeedY, SpeedX);
	
	rAngleX:= 50* COS(rAngle); //how much x increases
	rAngleY:= 50* SIN(rAngle); //how much y increases	
