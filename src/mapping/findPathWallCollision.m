function wallIdx = findPathWallCollision(path, wallBeliefs)
% FINDPATHWALLCOLLISION Detect if a planned path intersects any unchecked optional walls.
%
%   wallIdx = findPathWallCollision(path, wallBeliefs)
%
%   INPUTS
%       path         Kx2 matrix of [x, y] coordinates from the PRM findPath function.
%       wallBeliefs  Mx1 struct array of optional walls (with .status and .geometry).
%
%   OUTPUT
%       wallIdx      Index of the first 'unchecked' optional wall intersected by the path.
%                    Returns 0 if no intersection is found or if path is invalid.

wallIdx = 0;

% Basic check: a path must have at least 2 points to form a segment
if isempty(path) || size(path, 1) < 2
    return;
end

numSegments = size(path, 1) - 1;
numWalls = length(wallBeliefs);

% 1. Iterate through each segment of the robot's planned path
for i = 1:numSegments
    % Current path segment from point P(i) to P(i+1)
    x1 = path(i, 1);
    y1 = path(i, 2);
    x2 = path(i+1, 1);
    y2 = path(i+1, 2);
    
    % 2. Check this segment against every 'unchecked' optional wall
    for j = 1:numWalls
        % Only care about walls we haven't verified yet
        if strcmp(wallBeliefs(j).status, 'unchecked')
            
            % Get wall geometry [x3 y3 x4 y4]
            w = wallBeliefs(j).geometry;
            x3 = w(1); y3 = w(2);
            x4 = w(3); y4 = w(4);
            
            % 3. Use the provided intersectPoint function to check for collision
            [isect, ~, ~] = intersectPoint(x1, y1, x2, y2, x3, y3, x4, y4);
            
            if isect
                % Collision found! Return the index of this wall.
                wallIdx = j;
                return; % Exit immediately to handle the first collision encountered
            end
        end
    end
end

end