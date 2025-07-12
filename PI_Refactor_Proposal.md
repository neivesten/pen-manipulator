# PI Controller Refactor Proposal

The current motion blocks `FB_LinealMove` and `FB_CircularMove` only calculate
velocities from precomputed profiles.  They do not take the actual TCP
position into account, which can be seen in the time based calculations:

- Lines 33–80 in `FB_LinealMove.st` compute the velocity purely from the
  elapsed time `t` and the trapezoidal profile.
- Lines 120–158 in `FB_CircularMove.st` perform a similar time based
  computation for arcs.

This feedforward approach leads to accumulation of errors.  The TCP may stop
before the intended end point because the integrator in `PLC_PRG` uses the
computed velocity but never feeds back the position error.

## Proposed PI Based Controller

1. **Feedback**
   - Measure the current TCP position and calculate the error to the
     instantaneous target.  For a line this is the remaining vector to the
     end point; for an arc it is the difference to the point on the circle
     at the current angle.

2. **PI control per axis**
   - Implement a simple PI controller block operating on the X and Y axes.
   - `u = Kp * error + Ki * integral(error)`
   - Saturate `u` to the configured maximum speed.  Acceleration limits can
     be applied by rate limiting `u` between cycles.

3. **Linear moves**
   - The set point is simply the final point of the segment.  When called at
     each cycle the controller drives the TCP towards this point.  When the
     error norm is below a tolerance the move is complete.

4. **Circular moves**
   - Maintain the current angle along the circle as an internal state.
   - At each cycle compute the target X/Y from this angle and feed the
     position error into the PI controller.
   - Advance the angle based on the commanded tangential speed so that the
     TCP follows the path while correcting radial deviations.

5. **Integration into `PLC_PRG`**
   - Replace the velocity integration with calls to the PI controller for
     the active segment.  The returned velocities are integrated the same
     way as today but now contain feedback.

This approach keeps the existing max speed and acceleration parameters while
providing closed loop correction of the TCP position.  Starting from a PI
controller offers a simple implementation and can be extended to a full PID
if required.
