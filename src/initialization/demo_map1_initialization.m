function result = demo_map1_initialization()
% DEMO_MAP1_INITIALIZATION Example of initial localization on map1_3credits.
%
%   This script loads the first practice map, synthesizes one depth scan
%   from a known start pose, then recovers the pose using waypoint-depth
%   matching. It is intended as a minimal integration check for the final
%   project initialization stack.

baseDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
mapPath = fullfile(baseDir, '3credits_practice', 'map1_3credits.mat');
data = load(mapPath);

sensorOrigin = [0.13; 0.0];
angles = linspace(-pi, pi, 72)';
headingHypotheses = linspace(-pi, pi, 36);

% Waypoint 5 is less ambiguous than waypoint 1 when only known walls are used.
truePose = [data.waypoints(5, 1); data.waypoints(5, 2); pi / 2];
zMeasured = depthPredict(truePose, data.map, sensorOrigin, angles, 10.0);

result = initializeFromWaypoints(data, zMeasured, headingHypotheses, sensorOrigin, angles);

fprintf('True pose      : [%.3f, %.3f, %.3f]\n', truePose(1), truePose(2), truePose(3));
fprintf('Recovered pose : [%.3f, %.3f, %.3f]\n', ...
    result.bestPose(1), result.bestPose(2), result.bestPose(3));
fprintf('Best waypoint index: %d\n', result.bestWaypointIdx);
fprintf('Best heading index : %d\n', result.bestHeadingIdx);
end
