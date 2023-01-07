# Code for Modeling, Simulation, GUI and Statistical Analysis of a Robot-guided Needle Placement

![Graphical abstract](/ReadmeFiles/GraphicalAbstract.png)

Resources and extra documentation for the manuscript "Needle Placement for Robot-assisted 3D Ultrasound-guided Breast Biopsy: A Preliminary Study" published in IEEE Latin America Transactions. The code is organized by the type of programming language used in the project in the following order `Python -> Matlab -> R`. The folder's description is as follows

1. **Python**: For a geometrical model of woman breast and basic dimensions for a breast holder device         
2. **Matlab**: 
    > **Matlab files**: Scripts for robot modeling (forward and inverse kinematics), breast modeling, trajectory generation, a collision-free path algorithm and multiple simulation.  
    > **Simulink files**: For implementing a robot-assisted biopsy environment based on *Simscape* and *Stateflow-Flowcharts*.  Simulation settings: Ode23t Solver.  
    > **App files**: For designing a Graphical User Interface for setting the basic configurations of a robot-assisted needle placement before a complete simulation.
3. **R**: For a statistical analysis and curves plotting.

## Requirements
- Matlab 2020b or later. All additional packages (only needed codes) were uploaded in this repository. 
  - [Robotics Toolbox for MATLAB or RVC](https://petercorke.com/toolboxes/robotics-toolbox/) from *Robotics, Vision and Control Fundamental algorithms in MATLAB: Second Edition* by Peter Corke.
  - [Modern robotics toolbox](https://github.com/NxRLab/ModernRobotics) from *Modern Robotics: Mechanics, Planning, and Control* (Kevin Lynch and Frank Park, Cambridge University Press 2017 and 2019).
- Python 9.6
- R 2.1 or later
- **Optional**. Solidworks 2019 for CAD modeling and exporting files into Matlab

## Screenshots

<div id="header" align="center">
  <img src="ReadmeFiles\holderConcept.png" width="400"/>
  <img src="ReadmeFiles\breastAHolderProfile.png" width="400"/>
</div>

<div id="header" align="center">
  <img src="ReadmeFiles\breastWorkspace.png" width="400"/>
  <img src="ReadmeFiles\breastHolders.png" width="400"/>
  <img src="ReadmeFiles\robotJoinLimits.png" width="350"/>
  <img src="ReadmeFiles\insertionSelection1.gif" width="450"/>
  <img src="ReadmeFiles\biopsySimulation.gif" width="450"/>
</div>