TYPE PathPoint :
STRUCT
    P1 : POS2D; // Start point
    P2 : POS2D; // Midpoint for arc or endpoint for line
    P3 : POS2D; // Endpoint for arc (ignored for line)
    IsArc : BOOL;
    End : BOOL;
END_STRUCT
END_TYPE