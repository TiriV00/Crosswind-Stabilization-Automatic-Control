## Crosswind Stabilization for highway bus


### Introduction
This project was developed as part of a university exam. The goal was to apply automatic controls into the automotive field by replicating existing ADAS based on automatic controls or create a new one. We choose to develop our own interpretation of the crosswind stabilization system used for highway vehicles
like cargo truks or long distances buses.

### Requirements
To run the reported code is necessary the use of MATLAB and Simulink. **Before working with these files, we highly recommend reading the report we wrote, named `Automatic_Control_Rovighi_Tiri_final`.**

## This GitHub repository contains:
- The already cited report that contains the full explaination of the project
- A `Datasheets` folder in which are stored the datasheets describing the sensors and metodologies cited in the report regading the measurement instruments.
- A `scripts` folder in which are store the `.m` and `.slx` files composing the project to be run on MATLAB and Simulink.

## Workflow
The workfow that must be followed for successfully run the simulations is:
1. Run the `symbolic_model_v10.m` file for creating the symbolic matrices.
2. Run the `model_main_v10.m` file for storing the variables and matrices into the MATLAB workspace.
3. Run the `test_sim_non_linear_control_ver1.slx` simulation file for run the actual simulation on the model.

### Authors
- [Federico Rovighi](https://github.com/federovighi) - MUNER, EEIV - ADE Master's Degree  
- [Valerio Tiri](https://github.com/TiriV00) - MUNER, EEIV - ADE Master's Degree  
