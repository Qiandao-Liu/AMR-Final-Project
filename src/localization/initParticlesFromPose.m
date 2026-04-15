function particles = initParticlesFromPose(initPose, numParticles, positionSigma, thetaSigma)
% INITPARTICLESFROMPOSE Initialize a compact particle cloud around a pose.

if nargin < 2 || isempty(numParticles)
    numParticles = 200;
end
if nargin < 3 || isempty(positionSigma)
    positionSigma = 0.08;
end
if nargin < 4 || isempty(thetaSigma)
    thetaSigma = 8 * pi / 180;
end

particles.poses = [ ...
    initPose(1) + positionSigma * randn(1, numParticles); ...
    initPose(2) + positionSigma * randn(1, numParticles); ...
    localWrapToPi(initPose(3) + thetaSigma * randn(1, numParticles))];
particles.weights = ones(1, numParticles) / numParticles;
end

function wrapped = localWrapToPi(angle)
wrapped = mod(angle + pi, 2 * pi) - pi;
end
