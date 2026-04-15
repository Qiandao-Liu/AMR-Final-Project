function result = runAStarPlannerTestSimulator(Robot, mapMatPath, opts)
% RUNASTARPLANNERTESTSIMULATOR PF + beacon localization with A* replanning.
%
%   This is a simulator test loop that:
%   1. Seeds PF from a hardcoded initial pose
%   2. Uses PF + RSDepth + optional beacon fusion for localization
%   3. Replans with A* from the current estimated pose to the next goal
%   4. Tracks local path waypoints with feedback linearization
%
%   Default path:
%       [-4.5, -3.5, 0.262] -> [-2.0, -3.5] -> [-2.0, -1.5]

baseDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(fullfile(baseDir, 'src')));

if nargin < 2 || isempty(mapMatPath)
    mapMatPath = fullfile(baseDir, '3credits_practice', 'map1_3credits.mat');
end
if nargin < 3
    opts = struct();
end

if ~isfield(opts, 'startPose')
    opts.startPose = [-4.5; -3.5; 0.262];
end
if ~isfield(opts, 'goalWaypoints')
    opts.goalWaypoints = [-2.0, -3.5; -2.0, -1.5];
end
if ~isfield(opts, 'closeEnough')
    opts.closeEnough = 0.20;
end
if ~isfield(opts, 'lookaheadDistance')
    opts.lookaheadDistance = 0.25;
end
if ~isfield(opts, 'epsilon')
    opts.epsilon = 0.15;
end
if ~isfield(opts, 'desiredSpeed')
    opts.desiredSpeed = 0.30;
end
if ~isfield(opts, 'maxTime')
    opts.maxTime = 60;
end
if ~isfield(opts, 'loopPause')
    opts.loopPause = 0.05;
end
if ~isfield(opts, 'wheel2Center')
    opts.wheel2Center = 0.13;
end
if ~isfield(opts, 'maxWheelVelocity')
    opts.maxWheelVelocity = 0.50;
end
if ~isfield(opts, 'maxAngularSpeed')
    opts.maxAngularSpeed = 1.0;
end
if ~isfield(opts, 'sensorOrigin')
    opts.sensorOrigin = [0.13; 0];
end
if ~isfield(opts, 'numParticles')
    opts.numParticles = 250;
end
if ~isfield(opts, 'processNoise')
    opts.processNoise = diag([0.01, 0.01, 0.008]);
end
if ~isfield(opts, 'measurementNoise')
    opts.measurementNoise = 0.05 * eye(10);
end
if ~isfield(opts, 'beaconSigma')
    opts.beaconSigma = 0.10;
end
if ~isfield(opts, 'beaconWeightFactor')
    opts.beaconWeightFactor = 3.0;
end
if ~isfield(opts, 'beaconSensorOrigin')
    opts.beaconSensorOrigin = [0; 0];
end
if ~isfield(opts, 'plotEvery')
    opts.plotEvery = 2;
end
if ~isfield(opts, 'replanEvery')
    opts.replanEvery = 5;
end
if ~isfield(opts, 'planResolution')
    opts.planResolution = 0.10;
end
if ~isfield(opts, 'robotInflation')
    opts.robotInflation = 0.22;
end
if ~isfield(opts, 'stayAwayInflation')
    opts.stayAwayInflation = 0.25;
end
if ~isfield(opts, 'recoveryBackupTime')
    opts.recoveryBackupTime = 0.6;
end
if ~isfield(opts, 'recoveryBackupSpeed')
    opts.recoveryBackupSpeed = -0.12;
end
if ~isfield(opts, 'recoveryTurnRate')
    opts.recoveryTurnRate = 0.5;
end
if ~isfield(opts, 'recoveryMaxAttempts')
    opts.recoveryMaxAttempts = 3;
end
if ~isfield(opts, 'recoveryCooldownSteps')
    opts.recoveryCooldownSteps = 8;
end

mapStruct = load(mapMatPath);
map = mapStruct.map;
beaconLoc = [];
if isfield(mapStruct, 'beaconLoc')
    beaconLoc = mapStruct.beaconLoc;
end
stayAwayPoints = zeros(0, 2);
if isfield(mapStruct, 'stayAwayPoints')
    stayAwayPoints = mapStruct.stayAwayPoints;
end

boundary = [min(map(:, [1, 3]), [], 'all'), ...
            min(map(:, [2, 4]), [], 'all'), ...
            max(map(:, [1, 3]), [], 'all'), ...
            max(map(:, [2, 4]), [], 'all')];
angles = linspace(27 * pi / 180, -27 * pi / 180, 10)';

global dataStore;
dataStore = struct( ...
    'truthPose', [], ...
    'odometry', [], ...
    'rsdepth', [], ...
    'bump', [], ...
    'beacon', [], ...
    'pfEstimate', [], ...
    'targetIdx', [], ...
    'visibleTags', [], ...
    'plannedPath', []);

state = struct();
state.particles = initParticlesFromPose(opts.startPose, opts.numParticles);
state.poseEstimate = opts.startPose;
particlesPre = [];

result = struct();
result.reachedAll = false;
result.finalPoseEstimate = opts.startPose;
result.visitedWaypoints = 0;
result.recoveryCount = 0;

noRobotCount = 0;
goalIdx = 1;
iter = 0;
currentPath = zeros(0, 2);
pathTargetIdx = 1;
recoveryAttempts = 0;
recoveryCooldown = 0;

fig = figure('Name', 'A* Planner Test', 'Color', 'w');
cleanupObj = onCleanup(@() localCleanup(Robot));

SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.2);

tic;
while toc < opts.maxTime
    iter = iter + 1;
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

    if noRobotCount >= 5
        warning('Robot lost by overhead localization repeatedly. Stopping planner test.');
        break;
    end
    if isempty(dataStore.odometry) || isempty(dataStore.rsdepth)
        pause(opts.loopPause);
        continue;
    end

    latestOdom = dataStore.odometry(end, 2:3)';
    latestDepth = dataStore.rsdepth(end, 2:end)';
    tags = RealSenseTag(Robot);

    [state, particlesPre] = localizeStepPF( ...
        state, latestOdom, latestDepth, map, opts.sensorOrigin, angles, ...
        struct('processNoise', opts.processNoise, ...
               'measurementNoise', opts.measurementNoise, ...
               'tags', tags, ...
               'beaconLoc', beaconLoc, ...
               'beaconSigma', opts.beaconSigma, ...
               'beaconWeightFactor', opts.beaconWeightFactor, ...
               'beaconSensorOrigin', opts.beaconSensorOrigin));

    poseEst = state.poseEstimate;
    dataStore.pfEstimate = [dataStore.pfEstimate; toc, poseEst'];
    dataStore.targetIdx = [dataStore.targetIdx; toc, goalIdx];
    dataStore.visibleTags = [dataStore.visibleTags; toc, size(tags, 1)];

    latestBump = false;
    if ~isempty(dataStore.bump)
        latestBump = any(dataStore.bump(end, 2:end));
    end
    if latestBump
        fprintf('Recovery triggered: bump detected.\n');
        [state, particlesPre, recovered] = localRecover(Robot, mapMatPath, opts, goalIdx);
        if ~recovered
            warning('Recovery failed after bump. Stopping.');
            break;
        end
        recoveryAttempts = recoveryAttempts + 1;
        result.recoveryCount = recoveryAttempts;
        currentPath = zeros(0, 2);
        pathTargetIdx = 1;
        recoveryCooldown = opts.recoveryCooldownSteps;
        if recoveryAttempts >= opts.recoveryMaxAttempts
            warning('Exceeded maximum recovery attempts. Stopping.');
            break;
        end
        pause(opts.loopPause);
        continue;
    end

    currentGoal = opts.goalWaypoints(goalIdx, :);
    distToGoal = norm(currentGoal(:) - poseEst(1:2));
    if distToGoal <= opts.closeEnough
        fprintf('Reached planner goal %d at [%.3f, %.3f]\n', ...
            goalIdx, currentGoal(1), currentGoal(2));
        goalIdx = goalIdx + 1;
        result.visitedWaypoints = goalIdx - 1;
        currentPath = zeros(0, 2);
        pathTargetIdx = 1;

        if goalIdx > size(opts.goalWaypoints, 1)
            result.reachedAll = true;
            break;
        end

        currentGoal = opts.goalWaypoints(goalIdx, :);
    end

    plannerOpts = struct('resolution', opts.planResolution, ...
                         'inflationRadius', opts.robotInflation, ...
                         'stayAwayPoints', stayAwayPoints, ...
                         'stayAwayRadius', opts.stayAwayInflation);

    [~, ~, planningGrid, ~] = planPathAStar(map, boundary, poseEst(1:2)', currentGoal, plannerOpts);
    if recoveryCooldown <= 0 && localPoseInvalid(poseEst(1:2)', planningGrid)
        fprintf('Recovery triggered: PF pose lies in occupied/inflated region.\n');
        [state, particlesPre, recovered] = localRecover(Robot, mapMatPath, opts, goalIdx);
        if ~recovered
            warning('Recovery failed from invalid pose. Stopping.');
            break;
        end
        recoveryAttempts = recoveryAttempts + 1;
        result.recoveryCount = recoveryAttempts;
        currentPath = zeros(0, 2);
        pathTargetIdx = 1;
        recoveryCooldown = opts.recoveryCooldownSteps;
        if recoveryAttempts >= opts.recoveryMaxAttempts
            warning('Exceeded maximum recovery attempts. Stopping.');
            break;
        end
        pause(opts.loopPause);
        continue;
    end

    if isempty(currentPath) || mod(iter, opts.replanEvery) == 1
        [currentPath, ~, grid, found] = planPathAStar(map, boundary, poseEst(1:2)', currentGoal, plannerOpts); %#ok<ASGLU>
        if ~found || isempty(currentPath)
            fprintf('Recovery triggered: A* failed to find a path.\n');
            [state, particlesPre, recovered] = localRecover(Robot, mapMatPath, opts, goalIdx);
            if ~recovered
                warning('Recovery failed after no-path event. Stopping.');
                break;
            end
            recoveryAttempts = recoveryAttempts + 1;
            result.recoveryCount = recoveryAttempts;
            currentPath = zeros(0, 2);
            pathTargetIdx = 1;
            recoveryCooldown = opts.recoveryCooldownSteps;
            if recoveryAttempts >= opts.recoveryMaxAttempts
                warning('Exceeded maximum recovery attempts. Stopping.');
                break;
            end
            pause(opts.loopPause);
            continue;
        end
        dataStore.plannedPath = currentPath;
        pathTargetIdx = 1;
    end

    while pathTargetIdx < size(currentPath, 1) && ...
            norm(currentPath(pathTargetIdx, :)' - poseEst(1:2)) < opts.lookaheadDistance
        pathTargetIdx = pathTargetIdx + 1;
    end

    localTarget = currentPath(pathTargetIdx, :);
    delta = localTarget(:) - poseEst(1:2);
    distToLocal = norm(delta);

    if distToLocal < 1e-6
        desiredVel = [0; 0];
    else
        speed = opts.desiredSpeed;
        if distToLocal < 0.4
            speed = speed * max(distToLocal / 0.4, 0.25);
        end
        desiredVel = speed * delta / distToLocal;
    end

    [cmdV, cmdW] = feedbackLin(desiredVel(1), desiredVel(2), poseEst(3), opts.epsilon);
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, opts.maxWheelVelocity, opts.wheel2Center);
    cmdW = max(min(cmdW, opts.maxAngularSpeed), -opts.maxAngularSpeed);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

    if mod(iter, opts.plotEvery) == 0 && isvalid(fig)
        localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, opts.goalWaypoints, goalIdx, ...
            currentPath, pathTargetIdx, particlesPre, dataStore);
    end

    if mod(iter, 10) == 0
        fprintf('Pose=[%.3f %.3f %.3f], goal=%d, localPathIdx=%d, tags=%d\n', ...
            poseEst(1), poseEst(2), poseEst(3), goalIdx, pathTargetIdx, size(tags, 1));
    end

    if recoveryCooldown > 0
        recoveryCooldown = recoveryCooldown - 1;
    end

    pause(opts.loopPause);
end

SetFwdVelAngVelCreate(Robot, 0, 0);
result.finalPoseEstimate = state.poseEstimate;

if isvalid(fig)
    localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, opts.goalWaypoints, ...
        min(goalIdx, size(opts.goalWaypoints, 1)), currentPath, pathTargetIdx, particlesPre, dataStore);
end

if result.reachedAll
    fprintf('Completed A* planner closed-loop test.\n');
else
    fprintf('Stopped before completing all A* goals.\n');
end
fprintf('Final PF estimate: [x=%.3f, y=%.3f, th=%.3f]\n', ...
    result.finalPoseEstimate(1), result.finalPoseEstimate(2), result.finalPoseEstimate(3));
end

function localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, goalWaypoints, goalIdx, currentPath, pathTargetIdx, particlesPre, dataStore)
figure(fig);
clf(fig);
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');

for i = 1:size(map, 1)
    plot(ax, [map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'k-', 'LineWidth', 2);
end

if ~isempty(beaconLoc)
    plot(ax, beaconLoc(:, 2), beaconLoc(:, 3), 'ms', 'MarkerFaceColor', 'm', 'MarkerSize', 7);
end
if ~isempty(stayAwayPoints)
    plot(ax, stayAwayPoints(:, 1), stayAwayPoints(:, 2), 'x', 'Color', [1, 0.5, 0], 'MarkerSize', 8, 'LineWidth', 1.5);
end

plot(ax, goalWaypoints(:, 1), goalWaypoints(:, 2), 'bo', 'MarkerFaceColor', 'c', 'MarkerSize', 8);
plot(ax, goalWaypoints(goalIdx, 1), goalWaypoints(goalIdx, 2), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 14);

if ~isempty(currentPath)
    plot(ax, currentPath(:, 1), currentPath(:, 2), 'b-', 'LineWidth', 1.2);
    plot(ax, currentPath(pathTargetIdx, 1), currentPath(pathTargetIdx, 2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
end

if ~isempty(particlesPre)
    scatter(ax, particlesPre.poses(1, :), particlesPre.poses(2, :), 10, particlesPre.weights, 'filled', 'MarkerFaceAlpha', 0.35);
end

if ~isempty(dataStore.pfEstimate)
    plot(ax, dataStore.pfEstimate(:, 2), dataStore.pfEstimate(:, 3), 'r-', 'LineWidth', 1.5);
    plot(ax, dataStore.pfEstimate(end, 2), dataStore.pfEstimate(end, 3), 'ro', 'MarkerFaceColor', 'r');
end

if ~isempty(dataStore.truthPose)
    plot(ax, dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'g--', 'LineWidth', 1.2);
    plot(ax, dataStore.truthPose(end, 2), dataStore.truthPose(end, 3), 'go', 'MarkerFaceColor', 'g');
end

numVisible = 0;
if ~isempty(dataStore.visibleTags)
    numVisible = dataStore.visibleTags(end, 2);
end
title(ax, sprintf('A* + PF + Beacon Fusion (visible tags: %d)', numVisible));
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
xlim(ax, [min(map(:, [1, 3]), [], 'all') - 0.5, max(map(:, [1, 3]), [], 'all') + 0.5]);
ylim(ax, [min(map(:, [2, 4]), [], 'all') - 0.5, max(map(:, [2, 4]), [], 'all') + 0.5]);
drawnow limitrate;
end

function localCleanup(Robot)
try
    SetFwdVelAngVelCreate(Robot, 0, 0);
catch
end
end

function invalid = localPoseInvalid(pointXY, grid)
if pointXY(1) < grid.boundary(1) || pointXY(1) > grid.boundary(3) || ...
        pointXY(2) < grid.boundary(2) || pointXY(2) > grid.boundary(4)
    invalid = true;
    return;
end
[~, col] = min(abs(grid.xCenters - pointXY(1)));
[~, row] = min(abs(grid.yCenters - pointXY(2)));
invalid = grid.occupancy(row, col);
end

function [state, particlesPre, recovered] = localRecover(Robot, mapMatPath, opts, goalIdx)
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.1);
SetFwdVelAngVelCreate(Robot, opts.recoveryBackupSpeed, 0);
pause(opts.recoveryBackupTime);
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.1);

initOpts = struct('turnRate', opts.recoveryTurnRate, ...
                  'loopPause', opts.loopPause, ...
                  'maxTurnAngle', 2 * pi, ...
                  'headingHypotheses', linspace(-pi, pi, 73), ...
                  'signatureBins', linspace(-pi, pi, 73), ...
                  'wheel2Center', opts.wheel2Center, ...
                  'maxWheelVelocity', opts.maxWheelVelocity, ...
                  'sensorOriginForMatch', [0; 0], ...
                  'scoreSigma', 0.20, ...
                  'verbose', true);
initOpts.headingHypotheses(end) = [];
initOpts.signatureBins(end) = [];

try
    initResult = runInitialLocalizationSimulator(Robot, mapMatPath, initOpts);
    % Flush odometry generated during backup + relocalization spin so the
    % next PF step starts from the new pose rather than replaying recovery.
    AngleSensorRoomba(Robot);
    DistanceSensorRoomba(Robot);
    state = struct();
    state.particles = initParticlesFromPose(initResult.bestPose, opts.numParticles);
    state.poseEstimate = initResult.bestPose;
    particlesPre = [];
    recovered = true;
    fprintf('Recovery succeeded. Reinitialized near waypoint %d while heading to goal %d.\n', ...
        initResult.bestWaypointIdx, goalIdx);
catch me
    warning('Relocalization failed: %s', me.message);
    state = struct();
    particlesPre = [];
    recovered = false;
end
end
