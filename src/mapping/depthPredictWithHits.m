function [depth, hitIndices] = depthPredictWithHits(robotPose, map, sensorOrigin, angles, maxRange)
% DEPTHPREDICTWITHHITS Predict depth and identify the closest hit segment.

if nargin < 5
    maxRange = 10.0;
end

xRobot = robotPose(1);
yRobot = robotPose(2);
thetaRobot = robotPose(3);

R = [cos(thetaRobot), -sin(thetaRobot); ...
     sin(thetaRobot),  cos(thetaRobot)];

sensorGlobal = [xRobot; yRobot] + R * sensorOrigin(:);
xSensor = sensorGlobal(1);
ySensor = sensorGlobal(2);

numAngles = length(angles);
depth = maxRange * ones(numAngles, 1);
hitIndices = zeros(numAngles, 1);

for i = 1:numAngles
    angleGlobal = thetaRobot + angles(i);
    xRayEnd = xSensor + maxRange * cos(angleGlobal);
    yRayEnd = ySensor + maxRange * sin(angleGlobal);
    minRange = inf;
    minIdx = 0;

    for j = 1:size(map, 1)
        [isect, xInt, yInt] = intersectPoint( ...
            xSensor, ySensor, xRayEnd, yRayEnd, ...
            map(j, 1), map(j, 2), map(j, 3), map(j, 4));

        if ~isect
            continue;
        end

        rangeToWall = hypot(xInt - xSensor, yInt - ySensor);
        if rangeToWall > 0 && rangeToWall < minRange
            minRange = rangeToWall;
            minIdx = j;
        end
    end

    if isfinite(minRange)
        depth(i) = minRange;
        hitIndices(i) = minIdx;
    end
end
end
