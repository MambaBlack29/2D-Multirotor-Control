clear;
close all;
clc;

addpath('utils');
addpath('trajectories');

controlhandle = @controller;

% Choose which trajectory you want to test with

trajhandle = @traj_sine;

[t, state] = simulation_2d(controlhandle, trajhandle);
