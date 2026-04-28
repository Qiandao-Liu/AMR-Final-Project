function wallBeliefs = initWallBeliefs(optWalls)
% INITWALLBELIEFS Initialize the belief structure for optional walls.
%
%   wallBeliefs = initWallBeliefs(optWalls)
%
%   INPUTS
%       optWalls     Mx4 matrix of optional walls [x1 y1 x2 y2]
%
%   OUTPUT
%       wallBeliefs  Struct array (size M) containing:
%                      .geometry     1x4 coordinates [x1 y1 x2 y2]
%                      .probPresent  Scalar probability (0 to 1)
%                      .observationCount  Number of times wall has been seen
%                      .status       'unknown', 'present', or 'absent'
%
%   Note: Initial probability is set to 0.5.

numWalls = size(optWalls, 1);
wallBeliefs = struct( ...
    'geometry', cell(numWalls, 1), ...
    'probPresent', 0.5, ...
    'observationCount', 0, ...
    'status', 'unknown');

for i = 1:numWalls
    wallBeliefs(i).geometry = optWalls(i, :);
end

end