%% Mapping Module Unit Test
% This script tests the Bayesian update logic for optional walls.

clear; clc; close all;

% --- 1. Setup Environment ---
% Known boundaries (a 10x10 room)
knownMap = [-5,-5, 5,-5; 5,-5, 5,5; 5,5, -5,5; -5,5, -5,-5];

% Two optional walls
% Wall 1: At x=2 (Actually PRESENT)
% Wall 2: At x=-2 (Actually ABSENT)
optWalls = [2, -1, 2, 1; ...
           -2, -1, -2, 1];

% Sensor parameters
angles = linspace(-27, 27, 21)' * pi/180;
sensorOrigin = [0; 0.08];
maxRange = 4.0;

% Initialize Beliefs
wallBeliefs = initWallBeliefs(optWalls);

% --- 2. Simulation Parameters ---
numSteps = 20; % Number of observations
sensorNoiseSigma = 0.05;

fprintf('--- Starting Mapping Test ---\n');

%% --- Scenario 1: Robot looking at Wall 1 (Present) ---
% Robot at [0, 0, 0] looking towards x=2
robotPose1 = [0; 0; 0]; 

% Ground Truth for Scenario 1: Map + Wall 1
trueMap1 = [knownMap; optWalls(1, :)];

fprintf('\nTesting Wall 1 (Should be PRESENT)...\n');
for i = 1:numSteps
    % Generate synthetic sensor data with noise
    [zTrue, ~] = depthPredictNew(robotPose1, trueMap1, sensorOrigin, angles, maxRange);
    zMeasured = zTrue + sensorNoiseSigma * randn(size(zTrue));
    
    % Update beliefs
    wallBeliefs = updateWallBeliefs(robotPose1, zMeasured, knownMap, optWalls, wallBeliefs, sensorOrigin, angles);
    
    if mod(i, 5) == 0
        fprintf('Step %d: Prob Wall 1 = %.3f, Status = %s\n', i, ...
            wallBeliefs(1).probPresent, wallBeliefs(1).status);
    end
end

%% --- Scenario 2: Robot looking at Wall 2 (Absent) ---
% Robot at [0, 0, pi] looking towards x=-2
robotPose2 = [0; 0; pi]; 

% Ground Truth for Scenario 2: Only Known Map (Wall 2 is missing)
trueMap2 = knownMap;

fprintf('\nTesting Wall 2 (Should be ABSENT)...\n');
for i = 1:numSteps
    % Generate synthetic sensor data with noise
    [zTrue, ~] = depthPredictNew(robotPose2, trueMap2, sensorOrigin, angles, maxRange);
    zMeasured = zTrue + sensorNoiseSigma * randn(size(zTrue));
    
    % Update beliefs
    wallBeliefs = updateWallBeliefs(robotPose2, zMeasured, knownMap, optWalls, wallBeliefs, sensorOrigin, angles);
    
    if mod(i, 5) == 0
        fprintf('Step %d: Prob Wall 2 = %.3f, Status = %s\n', i, ...
            wallBeliefs(2).probPresent, wallBeliefs(2).status);
    end
end

%% --- 3. Final Verification & Visualization ---
[navMap, plotData] = getConfirmedMap(knownMap, wallBeliefs);

figure('Name', 'Mapping Result');
hold on; grid on; axis equal;

% Plot Static Walls (Black)
for i = 1:size(plotData.static, 1)
    line(plotData.static(i, [1,3]), plotData.static(i, [2,4]), 'Color', 'k', 'LineWidth', 2);
end

% Plot Confirmed Walls (Green/Black - using Green here to distinguish in test)
if ~isempty(plotData.confirmed)
    for i = 1:size(plotData.confirmed, 1)
        line(plotData.confirmed(i, [1,3]), plotData.confirmed(i, [2,4]), 'Color', 'g', 'LineWidth', 3, 'LineStyle', '--');
    end
end

% Plot Unknown/Absent indicator
fprintf('\n--- Final Map Summary ---\n');
fprintf('NavMap contains %d walls (Known + Present + Unknown)\n', size(navMap, 1));
fprintf('Wall 1 status: %s\n', wallBeliefs(1).status);
fprintf('Wall 2 status: %s\n', wallBeliefs(2).status);

title('Mapping Test: Green (Confirmed), Black (Static)');
xlabel('X (m)'); ylabel('Y (m)');