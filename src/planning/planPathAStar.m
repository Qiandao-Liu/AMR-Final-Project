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
startIdx = nearestFreeGridCell(startIdx, grid.occupancy);
goalIdx = nearestFreeGridCell(goalIdx, grid.occupancy);

if isempty(startIdx) || isempty(goalIdx)
    pathWorld = zeros(0, 2);
    pathIdx = zeros(0, 2);
    found = false;
    return;
end

[pathIdx, found] = astarGrid(grid.occupancy, startIdx, goalIdx);

if ~found
    pathWorld = zeros(0, 2);
    return;
end

pathWorld = [grid.xCenters(pathIdx(:, 2))', grid.yCenters(pathIdx(:, 1))'];
pathWorld = simplifyPath(pathWorld, 0.15);
pathWorld = [startXY(:)'; pathWorld];

if norm(pathWorld(end, :) - goalXY(:)') > 1e-9
    pathWorld = [pathWorld; goalXY(:)'];
end
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

function idxFree = nearestFreeGridCell(idx, occupancy)
[ny, nx] = size(occupancy);
row = idx(1);
col = idx(2);

if row < 1 || row > ny || col < 1 || col > nx
    idxFree = [];
    return;
end

if ~occupancy(row, col)
    idxFree = idx;
    return;
end

[cols, rows] = meshgrid(1:nx, 1:ny);
freeMask = ~occupancy;
if ~any(freeMask(:))
    idxFree = [];
    return;
end

dist2 = (rows - row).^2 + (cols - col).^2;
dist2(~freeMask) = inf;
[bestDist2, bestLin] = min(dist2(:));

if isinf(bestDist2)
    idxFree = [];
    return;
end

[bestRow, bestCol] = ind2sub([ny, nx], bestLin);
idxFree = [bestRow, bestCol];
end
