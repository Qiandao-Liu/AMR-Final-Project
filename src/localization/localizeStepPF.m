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

g = @(pose, u) integrateOdom(pose, u(1), u(2));
h = @(pose) depthPredict(pose, map, sensorOrigin, angles, 10.0);

[state.particles, particlesPre] = PF( ...
    state.particles, odom, zDepth, g, h, opts.processNoise, opts.measurementNoise);

w = particlesPre.weights;
p = particlesPre.poses;

state.poseEstimate = [ ...
    sum(w .* p(1, :)); ...
    sum(w .* p(2, :)); ...
    atan2(sum(w .* sin(p(3, :))), sum(w .* cos(p(3, :))))];
end
