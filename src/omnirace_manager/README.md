# Omnirace manager package

## Omnirace telemetry node

what this node should do:
-  it starts as IDLE, it listens to tf, when the omnicar is in position 0,0,0 (x,y,theta) for more than 5 seconds, it goes into READY.
-  when READY it listens to the twist, if recieves a different than zero command then it goes into STARTED.
-  In started it continue listening to the tf and it tracks the waypoints.
- - a wp is considered reached WNEH the position error is less than 0.15m and the orientation error is less than 0.2 rad.
- - additionally, the telemetry node keep tracks of the position error wrt to the waypoints, to compute penalty. this penalty for the current wpis the minimum error computed in the time window between the previous wp (or start) and the next wp (or end)
- when the car reaches last wp AND the commanded speed is zero then the state is FINISHED. from here the car moves again (twist different than zero) the it goes to IDLE again