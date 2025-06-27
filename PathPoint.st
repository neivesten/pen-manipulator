//
// Describes a single entry in a motion path.
// P1 and P3 define the start and end points of the move.
// P2 is used for intermediate information: it is the end point when
// `IsArc` is FALSE or the mid point of an arc when `IsArc` is TRUE.
//
TYPE PathPoint :
STRUCT
    P1 : POS2D; // Start point
    P2 : POS2D; // Midpoint for arc or endpoint for line
    P3 : POS2D; // Endpoint for arc (ignored for line)
    IsArc : BOOL;
    End : BOOL;
END_STRUCT
END_TYPE
