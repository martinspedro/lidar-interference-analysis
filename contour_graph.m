%% Contour graph plotter for analyzing the
% Created for RECPAD paper
clear;
close all;
clc;

%% Data

% Interference Reults
z = [0.00100681, 0.000391761, 0.000254237, 0.000157293, 7.07995e-05;
     0.000305704, 0.000160641, 1.98663e-05, 0.00010637, 7.67505e-05;
     0.000181502, 8.19654e-05, 8.15461e-05, 0.000103312, 2.28868e-05;
     0.000755008, 0.000301876, 0.000149389, 6.04174e-05, 1.98761e-05;
     0.000101064, 4.11009e-05, 5.8341e-05, 2.48946e-05, 4.14827e-05;
     0.00164605, 0.000654599, 0.000279153, 0.000233277, 0.000214902;
     7.42924e-05, 1.4524e-05, 8.78702e-05, 5.79216e-05, 5.8607e-05;
     0.000643007, 0.000356898, 0.000243215, 0.000190694, 0.000171214;
     0.000133446, 2.37312e-05, 2.78359e-06, 5.55442e-05, 1.29394e-05;
     0.000170042, 5.06963e-05, 3.28689e-05, 4.65614e-05, 4.95273e-05;
     0.000182215, 8.26014e-05, 2.4451e-05, 3.47493e-05, 6.47576e-05;
     0.000394053, 0.000100207, 4.60926e-05, 3.24115e-05, 1.93456e-05]


x = 0.1:0.1:0.5;  % Voxel edge length in meters
y = 1:12;         % Distance between LiDARs in meters

% Levels interval/numbers to use in the contour graph
% levels = [10e-7 10e-5 5*10e-5 10e-4 5*10e-4 10e-3]
% levels = [0.0004 0.0008 0.0012]
levels = 10

%% Plot contour graph
[M, c] = contourf(x, y, z, levels, '--', 'ShowText','off', 'LineWidth', 0.01)
title('Voxel relative interference', 'FontSize', 20);
xlabel('Voxel grid edge length (m)', 'FontSize', 15)
ylabel('Distance between the LIDARs (m)', 'FontSize', 15)

% Modify x axis to use the voxel edge length
xticks(x)
xticklabels({'0.1', '0.2', '0.3', '0.4', '0.5'})

% Full jet colormap and no grid
colormap(jet)
colorbar
grid off
