function scores = scorePoseHypotheses(map, poses, zMeasured, sensorOrigin, angles, opts)
% SCOREPOSEHYPOTHESES Score candidate poses using depth-map matching.
%
%   scores = scorePoseHypotheses(map, poses, zMeasured, sensorOrigin, angles, opts)
%
%   INPUTS
%       map          Nx4 known-wall map
%       poses        3xN candidate poses
%       zMeasured    Kx1 measured depth vector
%       sensorOrigin 2x1 depth sensor origin in robot frame
%       angles       Kx1 beam angles
%       opts         optional struct:
%           .sigma       default 0.10
%           .maxRange    default 10.0
%           .nanPenalty  default log(0.01)
%           .beaconLoc          optional [id x y] map
%           .beaconObservations optional [id relX relY scanAngle]
%           .beaconSensorOrigin default [0; 0]
%           .beaconSigma        default 0.20
%           .beaconWeightFactor default 4.0
%
%   OUTPUT
%       scores       1xN log-likelihood scores

if nargin < 6
    opts = struct();
end

if ~isfield(opts, 'sigma')
    opts.sigma = 0.10;
end
if ~isfield(opts, 'maxRange')
    opts.maxRange = 10.0;
end
if ~isfield(opts, 'nanPenalty')
    opts.nanPenalty = log(0.01);
end
if ~isfield(opts, 'beaconLoc')
    opts.beaconLoc = [];
end
if ~isfield(opts, 'beaconObservations')
    opts.beaconObservations = zeros(0, 4);
end
if ~isfield(opts, 'beaconSensorOrigin')
    opts.beaconSensorOrigin = [0; 0];
end
if ~isfield(opts, 'beaconSigma')
    opts.beaconSigma = 0.20;
end
if ~isfield(opts, 'beaconWeightFactor')
    opts.beaconWeightFactor = 4.0;
end

numPoses = size(poses, 2);
scores = -inf(1, numPoses);

for i = 1:numPoses
    zExpected = depthPredict(poses(:, i), map, sensorOrigin, angles, opts.maxRange);
    logLikelihood = 0;

    for k = 1:length(zMeasured)
        if isnan(zMeasured(k))
            if zExpected(k) < opts.maxRange
                logLikelihood = logLikelihood + opts.nanPenalty;
            end
            continue;
        end

        if ~isfinite(zExpected(k))
            logLikelihood = logLikelihood + opts.nanPenalty;
            continue;
        end

        err = zMeasured(k) - zExpected(k);
        sigma = opts.sigma;
        logLikelihood = logLikelihood ...
            - log(sqrt(2 * pi) * sigma) ...
            - 0.5 * (err / sigma)^2;
    end

    beaconLikelihood = localBeaconLikelihood( ...
        poses(:, i), opts.beaconObservations, opts.beaconLoc, ...
        opts.beaconSensorOrigin, opts.beaconSigma);

    scores(i) = logLikelihood + opts.beaconWeightFactor * beaconLikelihood;
end
end

function logLikelihood = localBeaconLikelihood(basePose, observations, beaconLoc, sensorOrigin, sigma)
logLikelihood = 0;

if isempty(observations) || isempty(beaconLoc)
    return;
end

logNorm = -log(2 * pi * sigma^2);

for i = 1:size(observations, 1)
    tagId = observations(i, 1);
    beaconIdx = find(beaconLoc(:, 1) == tagId, 1);
    if isempty(beaconIdx)
        continue;
    end

    scanPose = basePose;
    scanPose(3) = localWrapToPi(basePose(3) + observations(i, 4));
    theta = scanPose(3);
    R = [cos(theta), -sin(theta); ...
         sin(theta),  cos(theta)];

    sensorGlobal = scanPose(1:2) + R * sensorOrigin(:);
    beaconGlobal = beaconLoc(beaconIdx, 2:3)';
    predictedRel = R' * (beaconGlobal - sensorGlobal);
    measuredRel = observations(i, 2:3)';
    residual = measuredRel - predictedRel;

    logLikelihood = logLikelihood + logNorm - 0.5 * sum((residual / sigma) .^ 2);
end
end

function wrapped = localWrapToPi(angle)
wrapped = mod(angle + pi, 2 * pi) - pi;
end
