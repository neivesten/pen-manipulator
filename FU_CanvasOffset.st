//
// Function block to translate coordinates from a source reference
// frame (e.g. visualisation) to a local canvas coordinate system.
// The output is simply the input position shifted by the provided
// offset.
//
FUNCTION FU_CanvasOffset : Pos2D
VAR_INPUT
    Source : Pos2D;      // Position in source coordinate system
    Offset : Pos2D;      // Offset of the canvas origin in source coordinates
END_VAR

FU_CanvasOffset.X := Source.X - Offset.X;
FU_CanvasOffset.Y := Source.Y - Offset.Y;
// End of FU_CanvasOffset
