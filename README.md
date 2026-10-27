# Interactive UAV Flight Simulator GUI (MATLAB)

## What this is
An interactive 2D UAV waypoint-navigation and obstacle-avoidance demonstration with a GUI, Start/Pause/Reset controls, speed slider, animated UAV marker, and simulation metrics.

## Requirements
MATLAB with `uifigure`, `uiaxes`, and `timer` support (recent MATLAB versions recommended). No specialist aerospace toolbox is intended to be required.

## Run
1. Extract the ZIP.
2. Open MATLAB and set Current Folder to this directory.
3. Run:
   ```matlab
   uav_flight_simulator_gui
   ```

## GUI controls
- **Start Simulation:** animate the UAV along the computed route.
- **Pause Simulation:** pause animation.
- **Reset Simulation:** return the UAV to the start.
- **Simulation speed:** adjust animation pacing.

## Metrics
- Planned path length
- Number of path steps
- Minimum distance from the path to obstacle boundaries
- Whether the computed endpoint meets the goal tolerance

## Important limitations
This is a simplified 2D educational demo using a potential-field steering method. Such methods can become trapped in local minima, and the code does not model aircraft dynamics, wind, sensor uncertainty, or certified safety logic. It is not intended to control a real UAV. Review behavior and metrics in MATLAB before presenting results.
