# Crosswind Stabilization for Highway Bus

## Introduction
This project was developed as part of a university exam. The goal was to apply **automatic control techniques** in the automotive field by either replicating existing ADAS based on automatic control or developing a new one.  
We chose to create our own interpretation of a **crosswind stabilization system** for highway vehicles, such as cargo trucks and long-distance buses.

## Requirements
To run the provided code, **MATLAB** and **Simulink** are required.  

**Before working with these files, we highly recommend reading the report** `Automatic_Control_Rovighi_Tiri_final`, which provides a complete explanation of the project.

## Repository Structure
This repository contains:  
- **Project Report**: A detailed document explaining the entire project.  
- **`Datasheets` folder**: Contains datasheets describing the sensors and methodologies mentioned in the report, particularly regarding the measurement instruments.  
- **`Scripts` folder**: Contains the `.m` and `.slx` files necessary to run the project on MATLAB and Simulink.  

## Workflow
To successfully run the simulations, follow these steps:  
1. Run the `symbolic_model_v10.m` file to generate the symbolic matrices.  
2. Run the `model_main_v10.m` file to store variables and matrices in the MATLAB workspace.  
3. Run the `test_sim_non_linear_control_ver1.slx` file to execute the actual simulation of the model.  

## Authors
- [Federico Rovighi](https://github.com/federovighi) - MUNER, EEIV - ADE Master's Degree  
- [Valerio Tiri](https://github.com/TiriV00) - MUNER, EEIV - ADE Master's Degree  

