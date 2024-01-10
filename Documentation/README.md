# IEA Wind 22-MW RWT: Tabular Excel Data

The information in this Excel file is intended to make the reference turbine quantities as accessible as possible.  It contains the following tabs of information:

  - *Overview*: Summary information taken from WindIO yaml file and WISDEM calculations of component mass values.
  - *Blade Geometry*: Airfoil spanwise position as well as chord, twist, pitch axis location, and prebend as defined in the WindIO yaml file.
  - *Airfoil Coordinates and Polars*: As defined in the WindIO yaml file.  See the yaml file for more comments about each of the polars and assumptions regarding transition modeling
  - *Blade Support Structure*: Position and thicknesses of spar caps, shear webs, and leading/trailing edge reinforcements as defined in the WindIO yaml file.
  - *Blade Shell Layup*: Blade outer shell composite layers at each airfoil station as defined in the WindIO yaml file.
  - *Shear Web Layup*: Shear web composite layers at each airfoil station as defined in the WindIO yaml file.
  - *Blade Structural Properties*: 6x6 mass and stiffness matrices along the blade span computed by BECAS and used in HAWC2 or OpenFAST BeamDyn
  - *Tower Properties*: Diameter, thickness, and elastic properties along tower length
  - *Material Properties*: Density, moduli, and failure data for the materials defined in the WindIO yaml file.
  - *Rotor Performance*: Power, thrust, torque, rotor speed, tip speed, and pitch settings along the power curve.  COMPUTED WITH WISDEM/AESOpt?
  - *Nacelle Mass Properties*: Mass, center of mass coordinates, and moments of inertia for nacelle components and nacelle summaries.  Computed with WISDEM
