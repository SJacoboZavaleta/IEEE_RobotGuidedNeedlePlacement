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