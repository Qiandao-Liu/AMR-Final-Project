function result = runInitialLocalizationSimulator(Robot, mapMatPath, opts)
% RUNINITIALLOCALIZATIONSIMULATOR Rotate in place, collect a 360 depth
% signature, and estimate the initial pose from waypoint candidates.
%
%   result = runInitialLocalizationSimulator(Robot)
%   result = runInitialLocalizationSimulator(Robot, mapMatPath)
%   result = runInitialLocalizationSimulator(Robot, mapMatPath, opts)
%
%   This function is intended to be called as an autonomous control
%   program from the Create simulator. It assumes the robot is already
%   placed at a start waypoint on the loaded map.
%
%   The script:
%   1. Rotates the robot in place for roughly one full turn
%   2. Uses angular odometry to label each depth sample by relative angle
%   3. Builds an omnidirectional depth signature using the center RSDepth ray
%   4. Calls initializeFromWaypoints to estimate [x, y, theta]
%   5. Prints the estimated pose in the command window
%
%   INPUTS
%       Robot       simulator CreateRobot object
%       mapMatPath  optional .mat map path, default map1_3credits.mat
%       opts        optional struct:
%                     .turnRate              default 0.5 rad/s
%                     .loopPause             default 0.05 s
%                     .maxTurnAngle          default 2*pi
%                     .headingHypotheses     default 72 bins in [-pi, pi)
%                     .signatureBins         default 72 bins in [-pi, pi)
%                     .wheel2Center          default 0.13 m
%                     .maxWheelVelocity      default 0.45 m/s
%                     .sensorOriginForMatch  default [0; 0]
%                     .scoreSigma            default 0.20
%                     .verbose               default true
%
%   OUTPUT
%       result      struct with:
%                     .bestPose
%                     .bestWaypointIdx
%                     .bestHeadingIdx
%                     .signatureAngles
%                     .signatureDepth
%                     .rawAngles
%                     .rawDepth

if nargin < 2 || isempty(mapMatPath)
    baseDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    mapMatPath = fullfile(baseDir, '3credits_practice', 'map1_3credits.mat');
end

if nargin < 3
    opts = struct();
end

if ~isfield(opts, 'turnRate')
    opts.turnRate = 0.5;
end
if ~isfield(opts, 'loopPause')
    opts.loopPause = 0.05;
end
if ~isfield(opts, 'maxTurnAngle')
    opts.maxTurnAngle = 2 * pi;
end
if ~isfield(opts, 'headingHypotheses')
    opts.headingHypotheses = linspace(-pi, pi, 73);
    opts.headingHypotheses(end) = [];
end
if ~isfield(opts, 'signatureBins')
    opts.signatureBins = linspace(-pi, pi, 73);
    opts.signatureBins(end) = [];
end
if ~isfield(opts, 'wheel2Center')
    opts.wheel2Center = 0.13;
end
if ~isfield(opts, 'maxWheelVelocity')
    opts.maxWheelVelocity = 0.45;
end
if ~isfield(opts, 'sensorOriginForMatch')
    % For a full-turn signature we approximate sensing from the robot center.
    opts.sensorOriginForMatch = [0; 0];
end
if ~isfield(opts, 'scoreSigma')
    opts.scoreSigma = 0.20;
end
if ~isfield(opts, 'verbose')
    opts.verbose = true;
end

mapStruct = load(mapMatPath);

if ~isfield(mapStruct, 'map') || ~isfield(mapStruct, 'waypoints')
    error('Map file must contain at least fields "map" and "waypoints".');
end

% Reset angular odometry before the scan begins.
AngleSensorRoomba(Robot);
DistanceSensorRoomba(Robot);

rawAngles = [];
rawDepth = [];
turnAngle = 0;

initialDepth = RealSenseDist(Robot);
if ~isempty(initialDepth)
    centerIdx = ceil(length(initialDepth) / 2);
    centerDepth = initialDepth(centerIdx);
    if isfinite(centerDepth) && centerDepth > 0
        rawAngles(end + 1, 1) = 0; %#ok<AGROW>
        rawDepth(end + 1, 1) = centerDepth; %#ok<AGROW>
    end
end

[cmdV, cmdW] = limitCmds(0, opts.turnRate, opts.maxWheelVelocity, opts.wheel2Center);
SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

cleanupObj = onCleanup(@() SetFwdVelAngVelCreate(Robot, 0, 0));

while turnAngle < opts.maxTurnAngle
    pause(opts.loopPause);

    dTheta = AngleSensorRoomba(Robot);
    turnAngle = turnAngle + abs(dTheta);

    depthArray = RealSenseDist(Robot);
    if isempty(depthArray)
        continue;
    end

    centerIdx = ceil(length(depthArray) / 2);
    centerDepth = depthArray(centerIdx);

    if ~isfinite(centerDepth) || centerDepth <= 0
        continue;
    end

    rawAngles(end + 1, 1) = localWrapToPi(turnAngle); %#ok<AGROW>
    rawDepth(end + 1, 1) = centerDepth; %#ok<AGROW>
end

SetFwdVelAngVelCreate(Robot, 0, 0);

if numel(rawDepth) < 8
    error('Not enough depth samples collected during initialization.');
end

[signatureAngles, signatureDepth] = buildSignature(rawAngles, rawDepth, opts.signatureBins);

scoreOpts = struct( ...
    'sigma', opts.scoreSigma, ...
    'maxRange', 10.0, ...
    'nanPenalty', log(0.01));

result = initializeFromWaypoints( ...
    mapStruct, signatureDepth, opts.headingHypotheses, ...
    opts.sensorOriginForMatch, signatureAngles, scoreOpts);

result.signatureAngles = signatureAngles;
result.signatureDepth = signatureDepth;
result.rawAngles = rawAngles;
result.rawDepth = rawDepth;

if opts.verbose
    fprintf('\nInitial localization complete.\n');
    fprintf('Estimated pose: [x=%.3f, y=%.3f, theta=%.3f rad]\n', ...
        result.bestPose(1), result.bestPose(2), result.bestPose(3));
    fprintf('Best waypoint index: %d\n', result.bestWaypointIdx);
    fprintf('Best heading index : %d\n', result.bestHeadingIdx);
end
end

function [signatureAngles, signatureDepth] = buildSignature(rawAngles, rawDepth, targetAngles)
% Bin the raw rotating scan into an evenly spaced omnidirectional signature.

signatureAngles = targetAngles(:);
signatureDepth = nan(size(signatureAngles));

for i = 1:length(signatureAngles)
    angleError = abs(localWrapToPi(rawAngles - signatureAngles(i)));
    [bestErr, idx] = min(angleError);

    if isempty(idx) || bestErr > pi / 18
        continue;
    end

    signatureDepth(i) = rawDepth(idx);
end
end

function anglesWrapped = localWrapToPi(angles)
anglesWrapped = mod(angles + pi, 2 * pi) - pi;
end
