# Code for Modeling, Simulation, GUI and Statistical Analysis of a Robot-guided Needle Placement

![Graphical abstract](/ProjectImages/GraphicalAbstract.png)

Resources and extra documentation for the manuscript "Needle Placement for Robot-assisted 3D Ultrasound-guided Breast Biopsy: A Preliminary Study" published in IEEE Latin America Transactions. The code is organized by the type of programming language used in the project in the following order `Python -> Matlab -> R`. The project hierarchy and folders description is as follows

1. **PYTHON\BreastDimension**: For a geometrical model of woman breast and basic dimensions for a breast holder device         
2. **MATLAB\Models**:
   1. `getRootDirectory.m`. Script for loading current working directory.
   2. `startup.m`. Script for adding all folder paths to Matlab. 
   3. **CAD_models**:
      1. Breast. Breast models for A, B, and C-cup sizes.
      2. Devices. Needle devices for core and fine needle biopsies.
   4. **Matlab_models**. Scripts for robot modeling (forward and inverse kinematics), breast modeling, trajectory generation, a collision-free path algorithm and multiple simulation.
      1. Data: Trajectory data for joint trajectory testing.
      2. RoboticsToolbox:
         1. ForApp. Function scripts for implementing the GUI application.
         2. ForMatlab. Function scripts for all Matlab-based simulations.
         3. ForSimulink. Functions script for all Simulink-based simulations.
         4. ModernRobotics. Based on Modern robotics toolbox by Kevin Lynch and Frank Park.
         5. rvctools. Robotics Toolbox for MATLAB or RVC by Peter Corke.
      3. Test
   5. **Simulink_models**. For implementing a robot-assisted biopsy environment based on *Simscape* and *Stateflow-Flowcharts*.  Simulation settings: Ode23t Solver.
      1. Data. `.mat` files for the GUI application, target biopsy points, breast holder models, Finite element breast models, breast dimensions, robot dimensions, robot kinematics, end effector waypoints and Simulink model workspace.      
      2. Extra. Figures for GUI application.
      3. GUI. For designing a Graphical User Interface (GUI) for setting the basic configurations of a robot-assisted needle placement before a complete simulation (`.mlapp`).
      4. Results. Outputs for multiple simulation (`.xls` files) and target biopsy points for each breast-cup size (`.mat`) 
      5. Scripts. For computing multiple simulations (`.m`) and biopsy targets samplings (`.m`) based on factorial-method design. 
      6. SIM. Simulation models (`.slx`) for GUI application and multiple simulation.
   6. **work**.
3. **R\SimulationAnalysis**. For a statistical analysis and curves plotting.
4. **ProjectImages**. Some manuscript images, figures and animation for the `README.m` file.

## Requirements
- Matlab 2020b or later. All additional packages (only needed codes) were uploaded in this repository. 
  - [Robotics Toolbox for MATLAB or RVC](https://petercorke.com/toolboxes/robotics-toolbox/) from *Robotics, Vision and Control Fundamental algorithms in MATLAB: Second Edition* by Peter Corke.
  - [Modern robotics toolbox](https://github.com/NxRLab/ModernRobotics) from *Modern Robotics: Mechanics, Planning, and Control* (Kevin Lynch and Frank Park, Cambridge University Press 2017 and 2019).
- Python 9.6
- R 2.1 or later
- **Optional**. Solidworks 2019 for CAD modeling and exporting files into Matlab

## Screenshots

<div id="header" align="center">
  <img src="ProjectImages\holderConcept.png" width="400"/>
  <img src="ProjectImages\breastAHolderProfile.png" width="400"/>
</div>

<div id="header" align="center">
  <img src="ProjectImages\breastWorkspace.png" width="400"/>
  <img src="ProjectImages\breastHolders.png" width="400"/>
  <img src="ProjectImages\robotJoinLimits.png" width="350"/>
  <img src="ProjectImages\insertionSelection1.gif" width="450"/>
  <img src="ProjectImages\biopsySimulation.gif" width="450"/>
</div>

## Instructions for running the GUI
1. Run `startup.m` to add all paths to Matlab and run without problems
2. Go to `Simulink_models\GUI\` and run `biopsyControl.mlapp` to open the application.
3. The GUI lets you select randoms biopsy targets, define needle insertion avoiding collisions, generate robot trajectory and save all data in order to simulate a particular biopsy case in Simulink.
   > The application use is interactively easy to follow. The bottoms were sequentially ordered according to: Get biopsy target, define needle insertion, set needle device, generate robot trajectory for the preplacement stage, save data, reload model Simulink variables, simulate and get errors report.
  
4. This robot environment in Simulink can be opened in the option `Open Model` inside the third tab (`Postplacement stage`) or directly by going to `Simulink_models\SIM\breastBiopsyApp.slx`. Then, playing the option `Simulate` to initialize and showing a visual simulation window (based on Simscape multibody toolbox).  
   > **NOTE** When opening the `breastBiopsyApp.slx` directly without running the application before, there will be a warning message about a failed load of model workspace variables. To avoid this, run all the application steps to create the correct file `simulinkWorkSpace.mat` and reload all needed model variables. Finally, go to `Model Explorer -> Model Hierarchy -> breastBiopsyApp -> Model Workspcae -> Workspace data`in Simulink, browse for the `simulinkWorkSpace.mat` file and save the model. This will eliminate the warning message, and the `breastBiopsyApp.slx` model can also be able to run with the previous loaded variables any time.

## Instructions for running a multiple simulation
1. Open the `multiplesSIM.mat` and run.
2. The results will be saved in `Simulink_models\Results\` in Excel files (i.e. `simulationA_FNA` was created by simulating a breast holder of A-size cup and a fine needle). 
    > The Simulink file associated to `multiplesSIM.mat` is `breastBiopsyMulti.slx` and always run in second plane. Therefore, there will be not a visual simulation. However, this could be modified.