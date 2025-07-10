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

## Drawing Full Circles

`FB_CircularMove` supports drawing a complete circle by specifying the same
coordinates for the start (`P1`) and end (`P3`) points.  The midpoint `P2`
still determines the plane of the circle and its direction of travel.
