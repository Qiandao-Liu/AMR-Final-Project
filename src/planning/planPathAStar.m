function [pathWorld, pathIdx, grid, found] = planPathAStar(map, boundary, startXY, goalXY, opts)
% PLANPATHASTAR Build a planning grid and compute an A* path in world coordinates.
%
%   [pathWorld, pathIdx, grid, found] = planPathAStar(map, boundary, ...
%       startXY, goalXY, opts)

if nargin < 5
    opts = struct();
end

if ~isfield(opts, 'resolution')
    opts.resolution = 0.10;
end
if ~isfield(opts, 'inflationRadius')
    opts.inflationRadius = 0.20;
end
if ~isfield(opts, 'stayAwayPoints')
    opts.stayAwayPoints = zeros(0, 2);
end
if ~isfield(opts, 'stayAwayRadius')
    opts.stayAwayRadius = 0.25;
end

grid = buildOccupancyGrid( ...
    map, boundary, opts.resolution, opts.inflationRadius, ...
    opts.stayAwayPoints, opts.stayAwayRadius);

startIdx = worldToGrid(startXY, grid);
goalIdx = worldToGrid(goalXY, grid);
[pathIdx, found] = astarGrid(grid.occupancy, startIdx, goalIdx);

if ~found
    pathWorld = zeros(0, 2);
    return;
end

pathWorld = [grid.xCenters(pathIdx(:, 2))', grid.yCenters(pathIdx(:, 1))'];
pathWorld = simplifyPath(pathWorld, 0.15);
end

function idx = worldToGrid(pointXY, grid)
[~, col] = min(abs(grid.xCenters - pointXY(1)));
[~, row] = min(abs(grid.yCenters - pointXY(2)));
idx = [row, col];
end

function pathOut = simplifyPath(pathIn, spacing)
if size(pathIn, 1) <= 2
    pathOut = pathIn;
    return;
end

pathOut = pathIn(1, :);
lastKeep = pathIn(1, :);
for i = 2:size(pathIn, 1) - 1
    if norm(pathIn(i, :) - lastKeep) >= spacing
        pathOut = [pathOut; pathIn(i, :)]; %#ok<AGROW>
        lastKeep = pathIn(i, :);
    end
end
pathOut = [pathOut; pathIn(end, :)];
end
