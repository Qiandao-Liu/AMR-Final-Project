function [navMap, plotData] = getConfirmedMap(knownMap, wallBeliefs)
% GETCONFIRMEDMAP Generate a map for navigation and visualization.
%
%   [navMap, plotData] = getConfirmedMap(knownMap, wallBeliefs)
%
%   INPUTS
%       knownMap     Nx4 matrix of static walls [x1 y1 x2 y2]
%       wallBeliefs  Mx1 struct array of optional walls with status
%
%   OUTPUTS
%       navMap       Kx4 matrix for the Planner (Static + Present + Unknown)
%       plotData     Struct containing separated walls for visualization:
%                      .confirmed   (Confirmed present)
%                      .unknown     (Not yet determined)
%                      .absent      (Confirmed absent)

numOptWalls = length(wallBeliefs);

% 1. Separate optional walls by status
idxPresent = strcmp({wallBeliefs.status}, 'present');
idxUnknown = strcmp({wallBeliefs.status}, 'unknown');
idxAbsent  = strcmp({wallBeliefs.status}, 'absent');

% Extract geometries
optPresent = vertcat(wallBeliefs(idxPresent).geometry);
optUnknown = vertcat(wallBeliefs(idxUnknown).geometry);
optAbsent  = vertcat(wallBeliefs(idxAbsent).geometry);

% 2. Build the Navigation Map (Conservative Strategy)
% We include Unknown walls to avoid collisions until we are sure they are gone.
navMap = [knownMap; optPresent; optUnknown];

% 3. Prepare data for visualization (Requirement from Section 6)
% Black: Static + Confirmed Present
% Red: Unknown
% Not drawn: Absent
plotData = struct();
plotData.static = knownMap;
plotData.confirmed = optPresent;
plotData.unknown = optUnknown;
plotData.absent = optAbsent;

end