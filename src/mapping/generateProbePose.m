function probePose = generateProbePose(currentPose, wallGeom)
% GENERATEPROBEPOSE Calculate a pose to inspect an optional wall.
%
%   probePose = generateProbePose(currentPose, wallGeom)
%
%   INPUTS
%       currentPose  3x1 vector [x; y; theta] of the robot's current position.
%       wallGeom     1x4 vector [x1 y1 x2 y2] of the optional wall.
%
%   OUTPUT
%       probePose    3x1 vector [x; y; theta] for the robot to stop and 
%                    inspect the wall.

% --- Parameters ---
probeDist = 0.5; % Distance from the wall center to the robot center (m)

% 1. Calculate wall center (M)
mx = (wallGeom(1) + wallGeom(3)) / 2;
my = (wallGeom(2) + wallGeom(4)) / 2;

% 2. Calculate wall normal vector (n)
% Wall vector V = [x2-x1, y2-y1]
vx = wallGeom(3) - wallGeom(1);
vy = wallGeom(4) - wallGeom(2);

% Normal vectors are [-vy, vx] and [vy, -vx]
nx1 = -vy; ny1 = vx;
nx2 =  vy; ny2 = -vx;

% Normalize
len = hypot(vx, vy);
nx1 = nx1/len; ny1 = ny1/len;
nx2 = nx2/len; ny2 = ny2/len;

% 3. Determine which side the robot is currently on
% Vector from wall center to robot
dx = currentPose(1) - mx;
dy = currentPose(2) - my;

% Dot product to find the normal vector pointing towards the robot
if (dx * nx1 + dy * ny1) > (dx * nx2 + dy * ny2)
    bestN = [nx1; ny1];
else
    bestN = [nx2; ny2];
end

% 4. Calculate probe position (P)
% Robot center should be probeDist away from the wall center along the normal
px = mx + probeDist * bestN(1);
py = my + probeDist * bestN(2);

% 5. Calculate probe heading (theta)
% The robot should face the wall center (M)
% Vector from P to M
pToMx = mx - px;
pToMy = my - py;
ptheta = atan2(pToMy, pToMx);

probePose = [px; py; ptheta];

end