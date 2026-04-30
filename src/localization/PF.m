function [particles, particlesPre] = PF(particles, u, z, g, h, R, Q)
% PF One step of the particle filter.
%
%   [particles, particlesPre] = PF(particles, u, z, g, h, R, Q)
%
%   INPUTS
%       particles    struct with:
%                      .poses   3xN particle poses
%                      .weights 1xN normalized weights
%       u            2x1 odometry input [d; phi]
%       z            Kx1 sensor measurement
%       g            motion model handle, poseNext = g(pose, u)
%       h            measurement model handle, zHat = h(pose)
%       R            3x3 process noise covariance
%       Q            KxK measurement noise covariance
%
%   OUTPUTS
%       particles    resampled particle set with uniform weights
%       particlesPre predicted particles and normalized pre-resample weights

numParticles = size(particles.poses, 2);
sqrtR = chol(R, 'lower');

predictedPoses = zeros(3, numParticles);
for i = 1:numParticles
    poseDet = g(particles.poses(:, i), u);
    predictedPoses(:, i) = poseDet + sqrtR * randn(3, 1);
end

logWeights = zeros(1, numParticles);
useDiagonalQ = ismatrix(Q) && size(Q, 1) == size(Q, 2) && ...
    all(abs(Q(~eye(size(Q)))) < 1e-12, 'all');
if useDiagonalQ
    qDiag = diag(Q);
end

for i = 1:numParticles
    zHat = h(predictedPoses(:, i));
    valid = isfinite(z) & isfinite(zHat);

    if ~any(valid)
        logWeights(i) = -Inf;
        continue;
    end

    residual = z(valid) - zHat(valid);
    if useDiagonalQ
        logWeights(i) = -0.5 * sum((residual .^ 2) ./ qDiag(valid));
    else
        Qv = Q(valid, valid);
        logWeights(i) = -0.5 * (residual' * (Qv \ residual));
    end
end

maxLogWeight = max(logWeights);
if ~isfinite(maxLogWeight)
    weights = ones(1, numParticles) / numParticles;
else
    weights = exp(logWeights - maxLogWeight);
    weights = weights / sum(weights);
end

particlesPre.poses = predictedPoses;
particlesPre.weights = weights;

resampledPoses = zeros(3, numParticles);
cumWeights = cumsum(weights);
step = 1 / numParticles;
u0 = rand() * step;
j = 1;

for i = 1:numParticles
    pointer = u0 + (i - 1) * step;
    while j < numParticles && cumWeights(j) < pointer
        j = j + 1;
    end
    resampledPoses(:, i) = predictedPoses(:, j);
end

particles.poses = resampledPoses;
particles.weights = ones(1, numParticles) / numParticles;
end
