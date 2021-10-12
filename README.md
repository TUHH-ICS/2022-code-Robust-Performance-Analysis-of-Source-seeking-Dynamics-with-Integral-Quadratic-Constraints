# Robust Performance Analysis of Source-Seeking Dynamics with Integral Quadratic Constraints

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5564716.svg)](https://doi.org/10.5281/zenodo.5564716) 

## General

This repository contains the simulation code to reproduce the tables and figures presented in

A. Datar and H. Werner, "Robust Performance Analysis of Source-Seeking Dynamics with Integral Quadratic Constraints", *Americal Control Conference*, 2022

The code has the following 5 entry points:
1. Quadrotor robustness analysis (corresponding to Fig.4 from the paper): quadrotor_robustness\main_quadrotor_example.m 
2. Gain tuning for quadrotor example (corresponding to Fig.5 from the paper): sweep_kp_kd\main_kd_sweep.m
3. Simulate quadrotor dynamics (corresponding to Fig.6 from the paper): simulate_quadrotor\simulate_dynamics.m
4. Non-minimum phase example robustness analysis (corresponding to Fig.7 from the paper): non_minimum_phase_example\main_non_minimum_phase_example.m
5. LPV examples robustness analysis (corresponding to Fig.8 and Fig.9): LPV_Uncertain_system_example\main_LPV_example.m

These will produce among others the material presented in the paper. 

## Prerequisites

To run the simulation files, some additional packages need to be installed first.
1. Install [CVX](http://cvxr.com/cvx/download/)

The simulation code in this repository was tested in the following environment:
* *Windows 10 Pro* Version 20H2
* *Matlab* 2021a
