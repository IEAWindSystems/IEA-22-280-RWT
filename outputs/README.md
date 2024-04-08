# IEA 22 MW RWT Loads

Loads, natural frequencies, damping, and performance metrics for the IEA 22 MW RWT are available in this directory, based on model definitions for HAWC2 and OpenFAST available in the repository.
Currently available results:

* 01_steady_states: Steady-state performance of the rotor evaluated without tilt and gravity, including unsteady aerodynamic effects.
* 02_stability: Aeroelastic frequencies and damping the rotor evaluated without tilt and gravity, including unsteady aerodynamic effects.
* 03_turbulent_wind: Loads and performance metrics from 6 seeds of DLC1.1 with yaw misalignment of 0/+8/-8 degrees. The full turbine system on top of the monopile is modeled, including unsteady aerodynamic effects.

These results are discussed in an upcoming TORQUE 2024 paper led by William Collier at DNV.