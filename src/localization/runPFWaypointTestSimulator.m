function result = runPFWaypointTestSimulator(Robot, mapMatPath, opts)
% RUNPFWAYPOINTTESTSIMULATOR Hardcoded PF localization + waypoint tracking test.
%
%   result = runPFWaypointTestSimulator(Robot)
%   result = runPFWaypointTestSimulator(Robot, mapMatPath)
%   result = runPFWaypointTestSimulator(Robot, mapMatPath, opts)
%
%   Default test:
%     start pose  = [-4.5, -3.5, 0.262]
%     waypoint 1  = [-2.0, -3.5]
%     waypoint 2  = [-2.0, -1.5]
%
%   The filter uses the hardcoded start pose only for initial particle
%   seeding. After that, pose is maintained by PF using odometry + RSDepth,
%   and additionally constrained by beacon observations when visible.

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
if ~isfield(opts, 'waypoints')
    opts.waypoints = [-2.0, -3.5; -2.0, -1.5];
end
if ~isfield(opts, 'closeEnough')
    opts.closeEnough = 0.20;
end
if ~isfield(opts, 'epsilon')
    opts.epsilon = 0.15;
end
if ~isfield(opts, 'desiredSpeed')
    opts.desiredSpeed = 0.18;
end
if ~isfield(opts, 'maxTime')
    opts.maxTime = 45;
end
if ~isfield(opts, 'loopPause')
    opts.loopPause = 0.05;
end
if ~isfield(opts, 'wheel2Center')
    opts.wheel2Center = 0.13;
end
if ~isfield(opts, 'maxWheelVelocity')
    opts.maxWheelVelocity = 0.45;
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
if ~isfield(opts, 'plotEvery')
    opts.plotEvery = 2;
end
if ~isfield(opts, 'beaconSigma')
    opts.beaconSigma = 0.20;
end
if ~isfield(opts, 'beaconSensorOrigin')
    opts.beaconSensorOrigin = [0; 0];
end

mapStruct = load(mapMatPath);
map = mapStruct.map;
beaconLoc = [];
if isfield(mapStruct, 'beaconLoc')
    beaconLoc = mapStruct.beaconLoc;
end
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
    'visibleTags', []);

state = struct();
state.particles = initParticlesFromPose(opts.startPose, opts.numParticles);
state.poseEstimate = opts.startPose;
particlesPre = [];

result = struct();
result.reachedAll = false;
result.finalPoseEstimate = opts.startPose;
result.visitedWaypoints = 0;

noRobotCount = 0;
targetIdx = 1;
iter = 0;

fig = figure('Name', 'PF Waypoint Test', 'Color', 'w');

cleanupObj = onCleanup(@() localCleanup(Robot));

SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.2);

tic;
while toc < opts.maxTime
    iter = iter + 1;

    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

    if noRobotCount >= 5
        warning('Robot lost by overhead localization repeatedly. Stopping test.');
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
        delta = currentTarget(:) - poseEst(1:2);
        distToTarget = norm(delta);
    end

    if distToTarget < 1e-6
        desiredVel = [0; 0];
    else
        speed = opts.desiredSpeed;
        if distToTarget < 0.5
            speed = speed * max(distToTarget / 0.5, 0.25);
        end
        desiredVel = speed * delta / distToTarget;
    end

    [cmdV, cmdW] = feedbackLin(desiredVel(1), desiredVel(2), poseEst(3), opts.epsilon);
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, opts.maxWheelVelocity, opts.wheel2Center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

    if mod(iter, opts.plotEvery) == 0 && isvalid(fig)
        localUpdatePlot(fig, map, beaconLoc, opts.waypoints, targetIdx, particlesPre, dataStore);
    end

    if mod(iter, 10) == 0
        fprintf('PF estimate: [x=%.3f, y=%.3f, th=%.3f], target=%d, dist=%.3f, tags=%d\n', ...
            poseEst(1), poseEst(2), poseEst(3), targetIdx, distToTarget, size(tags, 1));
    end

    pause(opts.loopPause);
end

SetFwdVelAngVelCreate(Robot, 0, 0);

if isvalid(fig)
    localUpdatePlot(fig, map, beaconLoc, opts.waypoints, min(targetIdx, size(opts.waypoints, 1)), particlesPre, dataStore);
end

result.finalPoseEstimate = state.poseEstimate;

if result.reachedAll
    fprintf('Completed both hardcoded waypoint moves.\n');
else
    fprintf('Stopped before completing all waypoints.\n');
end
fprintf('Final PF estimate: [x=%.3f, y=%.3f, th=%.3f]\n', ...
    result.finalPoseEstimate(1), result.finalPoseEstimate(2), result.finalPoseEstimate(3));
end

function localUpdatePlot(fig, map, beaconLoc, waypoints, targetIdx, particlesPre, dataStore)
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

plot(ax, waypoints(:, 1), waypoints(:, 2), 'bo', 'MarkerFaceColor', 'c', 'MarkerSize', 8);
if ~isempty(waypoints)
    plot(ax, waypoints(targetIdx, 1), waypoints(targetIdx, 2), 'rp', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 14);
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

if ~isempty(dataStore.truthPose)
    plot(ax, dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'g--', 'LineWidth', 1.2);
    plot(ax, dataStore.truthPose(end, 2), dataStore.truthPose(end, 3), 'go', ...
        'MarkerFaceColor', 'g');
end

xlim(ax, [min(map(:, [1, 3]), [], 'all') - 0.5, max(map(:, [1, 3]), [], 'all') + 0.5]);
ylim(ax, [min(map(:, [2, 4]), [], 'all') - 0.5, max(map(:, [2, 4]), [], 'all') + 0.5]);
numVisible = 0;
if ~isempty(dataStore.visibleTags)
    numVisible = dataStore.visibleTags(end, 2);
end
title(ax, sprintf('PF + Beacon Fusion Test (visible tags: %d)', numVisible));
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
