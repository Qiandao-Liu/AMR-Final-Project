function d = segmentToSegmentDist(p1,p2,q1,q2)

    % sample along one segment (fast + sufficient)
    n = 10;
    d = inf;

    for i = 0:n
        t = i/n;
        p = (1-t)*p1 + t*p2;

        d = min(d, pointToSegmentDist(p,q1,q2));
    end
end