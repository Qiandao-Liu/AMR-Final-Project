function logLikelihood = beaconMeasurementLikelihood(pose, tags, beaconLoc, opts)
% BEACONMEASUREMENTLIKELIHOOD Log-likelihood of observed beacons for one pose.
%
%   logLikelihood = beaconMeasurementLikelihood(pose, tags, beaconLoc, opts)
%
%   INPUTS
%       pose       3x1 robot pose [x; y; theta]
%       tags       Mx5 array returned by RealSenseTag:
%                    [dt, id, x, y, rot]
%       beaconLoc  Bx3 array from compMap/map file:
%                    [id, x, y]
%       opts       optional struct:
%                    .positionSigma     default 0.20 m
%                    .sensorOrigin      default [0; 0]
%                    .unknownTagPenalty default log(0.01)
%
%   OUTPUT
%       logLikelihood  scalar log-likelihood

if nargin < 4
    opts = struct();
end

if ~isfield(opts, 'positionSigma')
    opts.positionSigma = 0.20;
end
if ~isfield(opts, 'sensorOrigin')
    opts.sensorOrigin = [0; 0];
end
if ~isfield(opts, 'unknownTagPenalty')
    opts.unknownTagPenalty = log(0.01);
end

if isempty(tags)
    logLikelihood = 0;
    return;
end

theta = pose(3);
R = [cos(theta), -sin(theta); ...
     sin(theta),  cos(theta)];
sensorGlobal = pose(1:2) + R * opts.sensorOrigin(:);
RGlobalToRobot = R';

sigma = opts.positionSigma;
logNorm = -log(2 * pi * sigma^2);
logLikelihood = 0;

for i = 1:size(tags, 1)
    tagId = tags(i, 2);
    beaconIdx = find(beaconLoc(:, 1) == tagId, 1);

    if isempty(beaconIdx)
        logLikelihood = logLikelihood + opts.unknownTagPenalty;
        continue;
    end

    beaconGlobal = beaconLoc(beaconIdx, 2:3)';
    predictedRel = RGlobalToRobot * (beaconGlobal - sensorGlobal);
    measuredRel = tags(i, 3:4)';
    residual = measuredRel - predictedRel;

    logLikelihood = logLikelihood + logNorm - 0.5 * sum((residual / sigma).^2);
end
end
