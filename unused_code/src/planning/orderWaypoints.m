function waypoints = orderWaypoints(points, pose)
% ORDERWAYPOINTS Orders points from closest to furthest from a given pose
%
% Inputs:
%   points - Nx2 array of [x y] points
%   pose   - 1x2 or 1x3 vector [x y (theta)]
%
% Output:
%   waypoints - Nx2 array of points ordered by distance from pose

    % Use only x, y from pose
    pos = pose(1:2);

    % Compute distances
    dists = vecnorm(points - pos, 2, 2);

    % Sort indices by distance
    [~, idx] = sort(dists);

    % Reorder points
    waypoints = points(idx, :);

end