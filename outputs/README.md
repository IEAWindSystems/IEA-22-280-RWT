# Steady states, normal operation, no tilt, no gravity

The steady state normal operation performance outputs are saved in a yaml file with a list of cases, one for each wind speed, with the following structure:

```yaml
   -  configuration:
          wind_speed: 3  # units: m/s
          rotor_speed: 1.8466210153489064  # units: rpm
          pitch: 0.5806431467239905  # nose down positive, units: deg
          tip_speed_ratio: 9.153211158238003  #  based on nominal radius of 142 m
      outputs:
          integrated:
              mechanical_power: 525934.8240476786  # units: W
              electrical_power: 490076.73  # units: W
              generator_torque: 2717658.7  # units: N*m
              rotor_thrust: 272229.21  # units: N
              rotor_torque: 2719728.4  # units: N*m
          distributed:
              aero_force_axial:
                  grid: [0.0, ..., 1.0]  # normalized grid
                  values: [0., ..., 0.]  # force normal to rotor plane, units: N/m
```

With the following content in the outputs:

| Parameter name                          | Description                                                                                     |
| --------------------------------------- | ----------------------------------------------------------------------------------------------- |
| rotor_speed                             | Rotor speed (rpm)                                                                               |
| pitch_angle                             | Pitch angle (deg) positive nose down                                                            |
| tip_speed_ratio                         | Tip speed ratio, based on a nominal radius of 142 m                                             |
| mechanical_power                        | Mechanical power (W)                                                                            |
| electrical_power                        | Electrical power (W)                                                                            |
| generator_torque                        | Generator torque (Nm)                                                                           |
| rotor_thrust                            | Thrust (N) along rotor rotational axis                                                          |
| rotor_torque                            | Torque (N*m) along rotor rotational axis                                                        |
| aero_force_axial                        | Applied aerodynamic axial force per unit length (N/m). Forces are perpendicular to rotor plane. |
| aero_force_tangential                   | Applied aerodynamic tangential force per unit length (N/m). Forces in rotor plane.              |
|                                         |
| blade_Fx_pitching                       | Internal blade x forces (N) in pitching system.                                                 |
| blade_Fy_pitching                       | Internal blade y forces (N) in pitching system.                                                 |
| blade_Fz_pitching                       | Internal blade z forces (N) in pitching system.                                                 |
| blade_Mx_pitching                       | Internal blade x bending moment (Nm) in pitching system.                                        |
| blade_My_pitching                       | Internal blade y bending moment (Nm) in pitching system.                                        |
| blade_Mz_pitching                       | Internal blade z bending moment (Nm) in pitching system.                                        |
| blade_translation_deflection_x_pitching | Blade x translational deflections (m). In blade root, pitching system                           |
| blade_translation_deflection_y_pitching | Blade y translational deflections (m). In blade root, pitching system                           |
| blade_translation_deflection_z_pitching | Blade z translational deflections (m). In blade root, pitching system                           |
| blade_rotation_deflection_x_pitching    | Blade x, y, z rotational deflections (deg). In blade root, pitching system                      |

The results can be plotted in the `plot_steady_states.yaml` file.