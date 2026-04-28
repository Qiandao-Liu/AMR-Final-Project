function [depth, hitIndices] = depthPredictNew(robotPose, map, sensorOrigin, angles, maxRange)
% DEPTHPREDICT Predict expected depth measurements and identify hit walls.
%
%   [depth, hitIndices] = depthPredict(robotPose, map, sensorOrigin, angles, maxRange)
%
%   INPUTS
%       robotPose    3x1 pose [x; y; theta] in global coordinates
%       map          Nx4 wall matrix [x1 y1 x2 y2] (can be static or optWalls)
%       sensorOrigin 2x1 sensor origin in robot frame [x; y]
%       angles       Kx1 beam angles in sensor frame (rad)
%       maxRange     optional scalar max range (default 10 m, use 3.0 for AMR)
%
%   OUTPUTS
%       depth        Kx1 expected ranges for each beam
%       hitIndices   Kx1 index of the wall in 'map' hit by each beam (0 if no hit)

if nargin < 5
    maxRange = 10.0;
end

xRobot = robotPose(1);
yRobot = robotPose(2);
thetaRobot = robotPose(3);

% Robot to Global rotation matrix
R = [cos(thetaRobot), -sin(thetaRobot); ...
     sin(thetaRobot),  cos(thetaRobot)];

% Sensor position in global coordinates
sensorGlobal = [xRobot; yRobot] + R * sensorOrigin(:);
xSensor = sensorGlobal(1);
ySensor = sensorGlobal(2);

numAngles = length(angles);
depth = maxRange * ones(numAngles, 1);
hitIndices = zeros(numAngles, 1);

% Trace each ray
for i = 1:numAngles
    angleGlobal = thetaRobot + angles(i);

    % Ray endpoint at max range
    xRayEnd = xSensor + maxRange * cos(angleGlobal);
    yRayEnd = ySensor + maxRange * sin(angleGlobal);

    minRange = inf;
    bestWallIdx = 0;

    % Check intersection with every wall in the provided map
    for j = 1:size(map, 1)
        [isect, xInt, yInt] = intersectPoint( ...
            xSensor, ySensor, xRayEnd, yRayEnd, ...
            map(j, 1), map(j, 2), map(j, 3), map(j, 4));

        if ~isect
            continue;
        end

        rangeToWall = hypot(xInt - xSensor, yInt - ySensor);

        % Update if this wall is the closest one seen by this ray so far
        if rangeToWall > 0 && rangeToWall < minRange
            minRange = rangeToWall;
            bestWallIdx = j;
        end
    end

    % Record results
    if isfinite(minRange)
        depth(i) = minRange;
        hitIndices(i) = bestWallIdx;
    end
end
end