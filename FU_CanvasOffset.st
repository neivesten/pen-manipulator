//
// Function block to translate coordinates from a source reference
// frame (e.g. visualisation) to a local canvas coordinate system.
// The output is simply the input position shifted by the provided
// offset.
//
FUNCTION_BLOCK FU_CanvasOffset
VAR_INPUT
    Source : Pos2D;      // Position in source coordinate system
    Offset : Pos2D;      // Offset of the canvas origin in source coordinates
END_VAR
VAR_OUTPUT
    Canvas : Pos2D;      // Transformed position relative to canvas origin
END_VAR

Canvas.X := Source.X - Offset.X;
Canvas.Y := Source.Y - Offset.Y;
// End of FU_CanvasOffset
