function navMap = buildOptimisticNavMap(mapStruct, wallBeliefs)
% BUILDOPTIMISTICNAVMAP Build a navigation map that ignores optional walls
% until they are confirmed present.

navMap = mapStruct.map;

if nargin < 2 || isempty(wallBeliefs)
    return;
end

idxPresent = strcmp({wallBeliefs.status}, 'present');
if any(idxPresent)
    navMap = [navMap; vertcat(wallBeliefs(idxPresent).geometry)];
end
end
