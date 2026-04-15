function depth = depthPredict(robotPose, map, sensorOrigin, angles, maxRange)
% DEPTHPREDICT Predict expected depth measurements from a pose and map.
%
%   depth = depthPredict(robotPose, map, sensorOrigin, angles, maxRange)
%
%   INPUTS
%       robotPose    3x1 pose [x; y; theta] in global coordinates
%       map          Nx4 wall matrix [x1 y1 x2 y2]
%       sensorOrigin 2x1 sensor origin in robot frame
%       angles       Kx1 beam angles in sensor frame
%       maxRange     optional scalar max range, default 10 m
%
%   OUTPUT
%       depth        Kx1 expected ranges

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

for i = 1:numAngles
    angleGlobal = thetaRobot + angles(i);

    xRayEnd = xSensor + maxRange * cos(angleGlobal);
    yRayEnd = ySensor + maxRange * sin(angleGlobal);

    minRange = inf;

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
        end
    end

    if isfinite(minRange)
        depth(i) = minRange;
    end
end
end
