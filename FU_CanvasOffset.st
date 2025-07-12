//
// Function used to translate coordinates from a local canvas
// reference frame to the global coordinate system.  The output
// is the input position shifted by the offset of the canvas in
// the global frame.
//
FUNCTION FU_CanvasOffset : Pos2D
VAR_INPUT
    Source : Pos2D;      // Position in source coordinate system
    Offset : Pos2D;      // Offset of the canvas origin in source coordinates
END_VAR

FU_CanvasOffset.X := Source.X + Offset.X;
FU_CanvasOffset.Y := Source.Y + Offset.Y;
// End of FU_CanvasOffset
