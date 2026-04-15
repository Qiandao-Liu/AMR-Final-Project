function [isect, x, y] = intersectPoint(x1, y1, x2, y2, x3, y3, x4, y4)
% INTERSECTPOINT Find the intersection point of two line segments.
%
%   [isect, x, y] = INTERSECTPOINT(x1, y1, x2, y2, x3, y3, x4, y4)
%   returns true if the segments intersect and the coordinates of the
%   intersection point.

x = [];
y = [];

denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

if abs(denom) < 1e-12
    isect = false;
    return;
end

ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

if ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1
    isect = true;
    x = x1 + ua * (x2 - x1);
    y = y1 + ua * (y2 - y1);
else
    isect = false;
end
end
