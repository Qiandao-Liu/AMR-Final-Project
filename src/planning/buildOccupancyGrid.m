function grid = buildOccupancyGrid(map, boundary, resolution, inflationRadius, stayAwayPoints, stayAwayRadius)
% BUILDOCCUPANCYGRID Build a binary planning grid from walls and stay-away points.
%
%   grid = buildOccupancyGrid(map, boundary, resolution, inflationRadius, ...
%       stayAwayPoints, stayAwayRadius)
%
%   INPUTS
%       map              Nx4 wall matrix [x1 y1 x2 y2]
%       boundary         [xmin ymin xmax ymax]
%       resolution       grid cell size (m)
%       inflationRadius  obstacle inflation radius (m)
%       stayAwayPoints   optional Mx2 points to avoid
%       stayAwayRadius   optional inflation radius for stay-away points (m)
%
%   OUTPUT
%       grid             struct with fields:
%                          .resolution
%                          .boundary
%                          .xCenters
%                          .yCenters
%                          .occupancy  (ny x nx logical)

if nargin < 5 || isempty(stayAwayPoints)
    stayAwayPoints = zeros(0, 2);
end
if nargin < 6 || isempty(stayAwayRadius)
    stayAwayRadius = inflationRadius;
end

xCenters = boundary(1):resolution:boundary(3);
yCenters = boundary(2):resolution:boundary(4);
[X, Y] = meshgrid(xCenters, yCenters);

occupancy = false(size(X));

% Mark cells close to any wall as occupied.
for i = 1:numel(X)
    p = [X(i), Y(i)];
    for j = 1:size(map, 1)
        d = pointToSegmentDistance(p, map(j, 1:2), map(j, 3:4));
        if d <= inflationRadius
            occupancy(i) = true;
            break;
        end
    end
end

% Mark cells near stay-away points as occupied as well.
for k = 1:size(stayAwayPoints, 1)
    d = hypot(X - stayAwayPoints(k, 1), Y - stayAwayPoints(k, 2));
    occupancy = occupancy | (d <= stayAwayRadius);
end

grid = struct();
grid.resolution = resolution;
grid.boundary = boundary;
grid.xCenters = xCenters;
grid.yCenters = yCenters;
grid.occupancy = occupancy;
end

function d = pointToSegmentDistance(p, a, b)
ab = b - a;
if all(abs(ab) < 1e-12)
    d = norm(p - a);
    return;
end
t = dot(p - a, ab) / dot(ab, ab);
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end
