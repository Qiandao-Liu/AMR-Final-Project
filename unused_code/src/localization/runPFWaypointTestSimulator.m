function result = runPFWaypointTestSimulator(Robot, mapMatPath, opts)
% RUNPFWAYPOINTTESTSIMULATOR Initialization + PF + A* sequential waypoint test.
%
%   result = runPFWaypointTestSimulator(Robot)
%   result = runPFWaypointTestSimulator(Robot, mapMatPath)
%   result = runPFWaypointTestSimulator(Robot, mapMatPath, opts)
%
%   Default test on map1:
%     robot starts near waypoint [-4.5, -3.5]
%     targets = mapStruct.waypoints in stored order
%
%   The test does not use ground-truth pose for control. It first runs the
%   initialization routine to estimate the start pose from waypoint
%   candidates, seeds PF from that estimate, then repeatedly plans an A*
%   path from the current PF estimate to the next waypoint and tracks the
%   local path with feedback linearization.

baseDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(fullfile(baseDir, 'src')));

if nargin < 2 || isempty(mapMatPath)
    mapMatPath = fullfile(baseDir, '3credits_practice', 'map1_3credits.mat');
end

if nargin < 3
    opts = struct();
end

mapStruct = load(mapMatPath);
map = mapStruct.map;
beaconLoc = [];
if isfield(mapStruct, 'beaconLoc')
    beaconLoc = mapStruct.beaconLoc;
end

if ~isfield(opts, 'startPose')
    opts.startPose = [-4.5; -3.5; 0.262];
end
if ~isfield(opts, 'waypoints')
    opts.waypoints = localResolveWaypointTargets(mapStruct);
end
if ~isfield(opts, 'closeEnough')
    opts.closeEnough = 0.20;
end
if ~isfield(opts, 'epsilon')
    opts.epsilon = 0.15;
end
if ~isfield(opts, 'desiredSpeed')
    opts.desiredSpeed = 0.30;
end
if ~isfield(opts, 'maxTime')
    opts.maxTime = 900000;
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
if ~isfield(opts, 'sensorOrigin')
    opts.sensorOrigin = [0.13; 0];
end
if ~isfield(opts, 'numParticles')
    opts.numParticles = 500;
end
if ~isfield(opts, 'processNoise')
    opts.processNoise = diag([0.004, 0.004, 0.003]);
end
if ~isfield(opts, 'measurementNoise')
    opts.measurementNoise = 0.08 * eye(10);
end
if ~isfield(opts, 'plotEvery')
    opts.plotEvery = 2;
end
if ~isfield(opts, 'beaconSigma')
    opts.beaconSigma = 0.05;
end
if ~isfield(opts, 'beaconWeightFactor')
    opts.beaconWeightFactor = 6.0;
end
if ~isfield(opts, 'beaconSensorOrigin')
    opts.beaconSensorOrigin = [0; 0];
end
if ~isfield(opts, 'maxAngularSpeed')
    opts.maxAngularSpeed = 1.0;
end
if ~isfield(opts, 'lookaheadDistance')
    opts.lookaheadDistance = 0.25;
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
if ~isfield(opts, 'initTurnRate')
    opts.initTurnRate = 0.5;
end
if ~isfield(opts, 'initMaxTurnAngle')
    opts.initMaxTurnAngle = 2 * pi;
end
if ~isfield(opts, 'initScoreSigma')
    opts.initScoreSigma = 0.20;
end

angles = linspace(27 * pi / 180, -27 * pi / 180, 10)';
stayAwayPoints = zeros(0, 2);
if isfield(mapStruct, 'stayAwayPoints')
    stayAwayPoints = mapStruct.stayAwayPoints;
end
boundary = [min(map(:, [1, 3]), [], 'all'), ...
            min(map(:, [2, 4]), [], 'all'), ...
            max(map(:, [1, 3]), [], 'all'), ...
            max(map(:, [2, 4]), [], 'all')];

global dataStore;
dataStore = struct( ...
    'odometry', [], ...
    'rsdepth', [], ...
    'bump', [], ...
    'beacon', [], ...
    'pfEstimate', [], ...
    'targetIdx', [], ...
    'visibleTags', [], ...
    'plannedPath', []);

state = struct();
particlesPre = [];

result = struct();
result.reachedAll = false;
result.finalPoseEstimate = [nan; nan; nan];
result.visitedWaypoints = 0;
result.initResult = struct();

iter = 0;
currentPath = zeros(0, 2);
pathTargetIdx = 1;

fig = figure('Name', 'PF Waypoint Test', 'Color', 'w');

cleanupObj = onCleanup(@() localCleanup(Robot));

SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.2);

if isempty(opts.waypoints)
    error('runPFWaypointTestSimulator:NoWaypoints', ...
        'No waypoint targets available for the waypoint test.');
end

initOpts = struct('turnRate', opts.initTurnRate, ...
                  'loopPause', opts.loopPause, ...
                  'maxTurnAngle', opts.initMaxTurnAngle, ...
                  'headingHypotheses', linspace(-pi, pi, 73), ...
                  'signatureBins', linspace(-pi, pi, 73), ...
                  'wheel2Center', opts.wheel2Center, ...
                  'maxWheelVelocity', opts.maxWheelVelocity, ...
                  'sensorOriginForMatch', [0; 0], ...
                  'scoreSigma', opts.initScoreSigma, ...
                  'verbose', true);
initOpts.headingHypotheses(end) = [];
initOpts.signatureBins(end) = [];

initResult = runInitialLocalizationSimulator(Robot, mapMatPath, initOpts);
result.initResult = initResult;

state.particles = initParticlesFromPose(initResult.bestPose, opts.numParticles);
state.poseEstimate = initResult.bestPose;
result.finalPoseEstimate = initResult.bestPose;

AngleSensorRoomba(Robot);
DistanceSensorRoomba(Robot);

targetIdx = localInitialTargetIndex(initResult.bestPose, opts.waypoints, opts.closeEnough);
result.visitedWaypoints = max(targetIdx - 1, 0);

if targetIdx > size(opts.waypoints, 1)
    result.reachedAll = true;
    result.visitedWaypoints = size(opts.waypoints, 1);
    result.finalPoseEstimate = state.poseEstimate;
    fprintf('Initialization pose is already within tolerance of all waypoint targets.\n');
    return;
end

tic;
while toc < opts.maxTime
    iter = iter + 1;

    dataStore = localReadSensors(Robot, dataStore);

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
    dataStore.targetIdx = [dataStore.targetIdx; toc, targetIdx];
    dataStore.visibleTags = [dataStore.visibleTags; toc, size(tags, 1)];

    currentTarget = opts.waypoints(targetIdx, :);
    delta = currentTarget(:) - poseEst(1:2);
    distToTarget = norm(delta);

    if distToTarget <= opts.closeEnough
        fprintf('Reached waypoint %d at [%.3f, %.3f]\n', ...
            targetIdx, currentTarget(1), currentTarget(2));
        targetIdx = targetIdx + 1;
        result.visitedWaypoints = targetIdx - 1;

        if targetIdx > size(opts.waypoints, 1)
            result.reachedAll = true;
            break;
        end

        currentTarget = opts.waypoints(targetIdx, :);
        currentPath = zeros(0, 2);
        pathTargetIdx = 1;
    end

    plannerOpts = struct('resolution', opts.planResolution, ...
                         'inflationRadius', opts.robotInflation, ...
                         'stayAwayPoints', stayAwayPoints, ...
                         'stayAwayRadius', opts.stayAwayInflation);

    if isempty(currentPath) || mod(iter, opts.replanEvery) == 1
        previousPath = currentPath;
        [candidatePath, ~, ~, found] = planPathAStar(map, boundary, poseEst(1:2)', currentTarget, plannerOpts);
        if found && ~isempty(candidatePath)
            currentPath = candidatePath;
            dataStore.plannedPath = currentPath;
            pathTargetIdx = 1;
        elseif isempty(previousPath)
            warning('A* failed to find an initial path from PF estimate to waypoint %d. Stopping test.', targetIdx);
            break;
        else
            warning('A* replanning failed at waypoint %d. Continuing on previous path.', targetIdx);
        end
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
        localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, opts.waypoints, targetIdx, ...
            currentPath, pathTargetIdx, particlesPre, dataStore);
    end

    if mod(iter, 10) == 0
        fprintf('PF estimate: [x=%.3f, y=%.3f, th=%.3f], target=%d, goalDist=%.3f, pathIdx=%d, tags=%d\n', ...
            poseEst(1), poseEst(2), poseEst(3), targetIdx, distToTarget, pathTargetIdx, size(tags, 1));
    end

    pause(opts.loopPause);
end

SetFwdVelAngVelCreate(Robot, 0, 0);

if isvalid(fig)
    localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, opts.waypoints, ...
        min(targetIdx, size(opts.waypoints, 1)), currentPath, pathTargetIdx, particlesPre, dataStore);
end

result.finalPoseEstimate = state.poseEstimate;

if result.reachedAll
    fprintf('Completed all %d waypoint targets.\n', size(opts.waypoints, 1));
else
    fprintf('Stopped before completing all waypoints.\n');
end
fprintf('Final PF estimate: [x=%.3f, y=%.3f, th=%.3f]\n', ...
    result.finalPoseEstimate(1), result.finalPoseEstimate(2), result.finalPoseEstimate(3));
end

function waypoints = localResolveWaypointTargets(mapStruct)
if isfield(mapStruct, 'waypoints') && ~isempty(mapStruct.waypoints)
    waypoints = mapStruct.waypoints;
else
    waypoints = zeros(0, 2);
end
end

function targetIdx = localInitialTargetIndex(startPose, waypoints, closeEnough)
targetIdx = 1;

if isempty(waypoints)
    return;
end

startPos = startPose(1:2)';
while targetIdx <= size(waypoints, 1) && norm(waypoints(targetIdx, :) - startPos) <= closeEnough
    targetIdx = targetIdx + 1;
end
end

function localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, waypoints, targetIdx, currentPath, pathTargetIdx, particlesPre, dataStore)
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
    plot(ax, beaconLoc(:, 2), beaconLoc(:, 3), 'ms', ...
        'MarkerFaceColor', 'm', 'MarkerSize', 7);
    for i = 1:size(beaconLoc, 1)
        text(ax, beaconLoc(i, 2) + 0.08, beaconLoc(i, 3) + 0.08, ...
            sprintf('%d', beaconLoc(i, 1)), 'Color', [0.5, 0, 0.5], 'FontSize', 8);
    end
end

if ~isempty(stayAwayPoints)
    plot(ax, stayAwayPoints(:, 1), stayAwayPoints(:, 2), 'x', ...
        'Color', [1, 0.5, 0], 'MarkerSize', 8, 'LineWidth', 1.5);
end

plot(ax, waypoints(:, 1), waypoints(:, 2), 'bo', 'MarkerFaceColor', 'c', 'MarkerSize', 8);
if ~isempty(waypoints)
    plot(ax, waypoints(targetIdx, 1), waypoints(targetIdx, 2), 'rp', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 14);
end

if ~isempty(currentPath)
    plot(ax, currentPath(:, 1), currentPath(:, 2), 'b-', 'LineWidth', 1.2);
    plot(ax, currentPath(pathTargetIdx, 1), currentPath(pathTargetIdx, 2), 'ko', ...
        'MarkerFaceColor', 'y', 'MarkerSize', 8);
end

if ~isempty(particlesPre)
    scatter(ax, particlesPre.poses(1, :), particlesPre.poses(2, :), ...
        10, particlesPre.weights, 'filled', 'MarkerFaceAlpha', 0.35);
end

if ~isempty(dataStore.pfEstimate)
    plot(ax, dataStore.pfEstimate(:, 2), dataStore.pfEstimate(:, 3), 'r-', 'LineWidth', 1.5);
    plot(ax, dataStore.pfEstimate(end, 2), dataStore.pfEstimate(end, 3), 'ro', ...
        'MarkerFaceColor', 'r');
end

xlim(ax, [min(map(:, [1, 3]), [], 'all') - 0.5, max(map(:, [1, 3]), [], 'all') + 0.5]);
ylim(ax, [min(map(:, [2, 4]), [], 'all') - 0.5, max(map(:, [2, 4]), [], 'all') + 0.5]);
numVisible = 0;
if ~isempty(dataStore.visibleTags)
    numVisible = dataStore.visibleTags(end, 2);
end
title(ax, sprintf('Initialization + PF + A* Test (visible tags: %d)', numVisible));
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
drawnow limitrate;
end

function localCleanup(Robot)
try
    SetFwdVelAngVelCreate(Robot, 0, 0);
catch
end
end

function dataStore = localReadSensors(Robot, dataStore)
try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

try
    deltaD = DistanceSensorRoomba(CreatePort);
    deltaA = AngleSensorRoomba(CreatePort);
    dataStore.odometry = [dataStore.odometry; toc, deltaD, deltaA];
catch
    disp('Error retrieving or saving odometry data.');
end

try
    [BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(CreatePort);
    dataStore.bump = [dataStore.bump; toc, ...
        BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront];
catch
    disp('Error retrieving or saving bump sensor data.');
end

try
    depthArray = RealSenseDist(Robot);
    dataStore.rsdepth = [dataStore.rsdepth; toc, depthArray'];
catch
    disp('Error retrieving or saving RealSense depth data.');
end

try
    tags = RealSenseTag(Robot);
    if ~isempty(tags)
        dataStore.beacon = [dataStore.beacon; repmat(toc, size(tags, 1), 1), tags];
    end
catch
    disp('Error retrieving or saving beacon data.');
end
end
