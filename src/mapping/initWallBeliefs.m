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
%                      .status       String: 'unchecked', 'confirmed_present', 
%                                    or 'confirmed_absent'
%
%   Note: The 'unchecked' status supports optimistic planning, where walls 
%         are assumed absent until the robot's planned path intersects them, 
%         triggering a dedicated probing action.

numWalls = size(optWalls, 1);

% Initialize the struct array. Using a cell for 'geometry' triggers 
% the broadcast creation of the struct array.
wallBeliefs = struct( ...
    'geometry', cell(numWalls, 1), ...
    'probPresent', 0.5, ...
    'status', 'unchecked');

for i = 1:numWalls
    wallBeliefs(i).geometry = optWalls(i, :);
end

end