function testWallCrossingPF()
% TESTWALLCROSSINGPF Verify impossible wall-crossing particle transitions are suppressed.

state.particles.poses = [
    -0.10,  0.10;
     0.00,  0.50;
     0.00,  0.00
];
state.particles.weights = [0.5, 0.5];

odom = [0; 0];
zDepth = 10;
map = [0, -1, 0, 1];
sensorOrigin = [0; 0];
angles = 0;

opts = struct();
opts.processNoise = 1e-12 * eye(3);
opts.measurementNoise = 100 * eye(1);
opts.wallCrossingMap = map;
opts.wallCrossingPenalty = 1e-6;

% Force the first particle to jump through the wall; leave the second on the same side.
state.particles.poses(:, 1) = [-0.10; 0.00; 0.00];
state.particles.poses(:, 2) = [0.10; 0.50; 0.00];
gShift = @(pose, ~) pose + [0.20; 0; 0];
h = @(pose) depthPredict(pose, map, sensorOrigin, angles, 10.0);
[~, particlesPre] = PF(state.particles, odom, zDepth, gShift, h, opts.processNoise, opts.measurementNoise);
particlesPre = localApplyGateForTest(state.particles, particlesPre, map, opts.wallCrossingPenalty);

assert(particlesPre.weights(1) < 1e-3);
assert(particlesPre.weights(2) > 0.99);
fprintf('testWallCrossingPF passed.\n');
end

function particlesPre = localApplyGateForTest(previousParticles, particlesPre, wallMap, crossingPenalty)
weights = particlesPre.weights;
for i = 1:size(particlesPre.poses, 2)
    [isect, xInt, yInt] = intersectPoint( ...
        previousParticles.poses(1, i), previousParticles.poses(2, i), ...
        particlesPre.poses(1, i), particlesPre.poses(2, i), ...
        wallMap(1), wallMap(2), wallMap(3), wallMap(4));
    if isect
        hitPoint = [xInt; yInt];
        if norm(hitPoint - previousParticles.poses(1:2, i)) > 1e-5 && ...
                norm(hitPoint - particlesPre.poses(1:2, i)) > 1e-5
            weights(i) = weights(i) * crossingPenalty;
        end
    end
end
particlesPre.weights = weights / sum(weights);
end
