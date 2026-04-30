function [state, particlesPre] = localizeStepPF(state, odom, zDepth, map, sensorOrigin, angles, opts)
% LOCALIZESTEPPF One online localization update for the final project.
%
%   [state, particlesPre] = localizeStepPF(state, odom, zDepth, map, ...
%       sensorOrigin, angles, opts)
%
%   INPUTS
%       state         struct with field .particles
%       odom          2x1 odometry input [d; phi]
%       zDepth        Kx1 measured depth vector
%       map           Nx4 known-wall map used for localization
%       sensorOrigin  2x1 sensor origin in robot frame
%       angles        Kx1 beam angles
%       opts          optional struct:
%                       .processNoise      default diag([0.01 0.01 0.005])
%                       .measurementNoise  default 0.05 * eye(K)
%                       .tags              default []
%                       .beaconLoc         default []
%                       .beaconSigma       default 0.20
%                       .beaconWeightFactor default 3.0
%                       .beaconSensorOrigin default [0; 0]
%
%   OUTPUTS
%       state         updated state with:
%                       .particles
%                       .poseEstimate      weighted mean pose before resample
%       particlesPre  weighted particles before resample

if nargin < 7
    opts = struct();
end

if ~isfield(opts, 'processNoise')
    opts.processNoise = diag([0.01, 0.01, 0.005]);
end
if ~isfield(opts, 'measurementNoise')
    opts.measurementNoise = 0.05 * eye(length(zDepth));
end
if ~isfield(opts, 'tags')
    opts.tags = [];
end
if ~isfield(opts, 'beaconLoc')
    opts.beaconLoc = [];
end
if ~isfield(opts, 'beaconSigma')
    opts.beaconSigma = 0.20;
end
if ~isfield(opts, 'beaconWeightFactor')
    opts.beaconWeightFactor = 3.0;
end
if ~isfield(opts, 'beaconSensorOrigin')
    opts.beaconSensorOrigin = [0; 0];
end
if ~isfield(opts, 'wallCrossingGate')
    opts.wallCrossingGate = true;
end
if ~isfield(opts, 'wallCrossingPenalty')
    opts.wallCrossingPenalty = 1e-6;
end
if ~isfield(opts, 'wallCrossingMap')
    opts.wallCrossingMap = map;
end

g = @(pose, u) integrateOdom(pose, u(1), u(2));
h = @(pose) depthPredict(pose, map, sensorOrigin, angles, 10.0);

previousParticles = state.particles;
[state.particles, particlesPre] = PF( ...
    state.particles, odom, zDepth, g, h, opts.processNoise, opts.measurementNoise);

if opts.wallCrossingGate && ~isempty(opts.wallCrossingMap)
    particlesPre = applyWallCrossingGate(previousParticles, particlesPre, opts.wallCrossingMap, opts.wallCrossingPenalty);
    state.particles = resampleParticles(particlesPre);
end

% If beacons are visible, apply an additional reweighting step using the
% tag-relative positions. If no beacons are visible, keep the depth-only PF.
if ~isempty(opts.tags) && ~isempty(opts.beaconLoc)
    numParticles = size(particlesPre.poses, 2);
    logBeaconWeights = zeros(1, numParticles);

    beaconOpts = struct( ...
        'positionSigma', opts.beaconSigma, ...
        'sensorOrigin', opts.beaconSensorOrigin, ...
        'unknownTagPenalty', log(0.01));

    for i = 1:numParticles
        logBeaconWeights(i) = beaconMeasurementLikelihood( ...
            particlesPre.poses(:, i), opts.tags, opts.beaconLoc, beaconOpts);
    end

    logDepthWeights = log(max(particlesPre.weights, realmin));
    logCombined = logDepthWeights + opts.beaconWeightFactor * logBeaconWeights;
    logCombined = logCombined - max(logCombined);
    combinedWeights = exp(logCombined);

    weightSum = sum(combinedWeights);
    if isfinite(weightSum) && weightSum > 0
        combinedWeights = combinedWeights / weightSum;
        particlesPre.weights = combinedWeights;
        state.particles = resampleParticles(particlesPre);
    end
end

w = particlesPre.weights;
p = particlesPre.poses;

state.poseEstimate = [ ...
    sum(w .* p(1, :)); ...
    sum(w .* p(2, :)); ...
    atan2(sum(w .* sin(p(3, :))), sum(w .* cos(p(3, :))))];
end

function particlesPre = applyWallCrossingGate(previousParticles, particlesPre, wallMap, crossingPenalty)
numParticles = size(particlesPre.poses, 2);
weights = particlesPre.weights;

for i = 1:numParticles
    if localSegmentCrossesWall(previousParticles.poses(1:2, i)', particlesPre.poses(1:2, i)', wallMap)
        weights(i) = weights(i) * crossingPenalty;
    end
end

weightSum = sum(weights);
if isfinite(weightSum) && weightSum > 0
    particlesPre.weights = weights / weightSum;
end
end

function crosses = localSegmentCrossesWall(p1, p2, wallMap)
crosses = false;
if norm(p2 - p1) < 1e-6
    return;
end

for i = 1:size(wallMap, 1)
    [isect, xInt, yInt] = intersectPoint( ...
        p1(1), p1(2), p2(1), p2(2), ...
        wallMap(i, 1), wallMap(i, 2), wallMap(i, 3), wallMap(i, 4));
    if isect
        hitPoint = [xInt, yInt];
        if norm(hitPoint - p1) > 1e-5 && norm(hitPoint - p2) > 1e-5
            crosses = true;
            return;
        end
    end
end
end

function particles = resampleParticles(particlesPre)
numParticles = size(particlesPre.poses, 2);
resampledPoses = zeros(3, numParticles);
cumWeights = cumsum(particlesPre.weights);
step = 1 / numParticles;
u0 = rand() * step;
j = 1;

for i = 1:numParticles
    pointer = u0 + (i - 1) * step;
    while j < numParticles && cumWeights(j) < pointer
        j = j + 1;
    end
    resampledPoses(:, i) = particlesPre.poses(:, j);
end

particles.poses = resampledPoses;
particles.weights = ones(1, numParticles) / numParticles;
end
