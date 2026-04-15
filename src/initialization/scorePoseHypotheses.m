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

    scores(i) = logLikelihood;
end
end
