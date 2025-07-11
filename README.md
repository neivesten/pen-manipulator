# 3D Pen Manipulator in CtrlX PLC Engineering

This repository contains a small example project written in Structured
Text (ST) for **ctrlX PLC Engineering**.  The code simulates a simple
2‑D pen manipulator capable of moving along straight lines and circular
arcs.  The program demonstrates how to build basic motion blocks using
trapezoidal and circular motion profiles.

The project can be imported into a ctrlX PLC Engineering workspace.  It
is mainly intended as a learning resource, so the motion logic is kept
minimal and easy to read.

## Accessing and Initialising Speed

`FB_TrapezoidalMove` and `FB_CircularMove` allow their `SpeedX` and
`SpeedY` values to be passed in as `VAR_IN_OUT` parameters.  These inputs
provide the initial velocity of a new motion segment and are updated on
every call.  You can reset these variables to zero when restarting the
simulation or feed them from the previous segment to achieve continuous
motion.

## Coordinate Systems

The program works with three coordinate spaces:

1. **Global** – the absolute frame used by the visualisation. All final
   positions are expressed here.
2. **Relative/Home** – a frame whose origin is the "home" position of the
   tool.  The manipulator always starts and ends at this location.
3. **Canvas** – a local frame describing the drawing canvas. The path
   points contained in `Path` are specified relative to this origin.

## Canvas Coordinate Translation

`FU_CanvasOffset` converts a position from canvas coordinates to global
coordinates by adding the offset of the canvas origin.  Use it whenever a
`PathPoint` needs to be expressed in the global frame.
