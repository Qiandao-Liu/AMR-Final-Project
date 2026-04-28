function d = pointToSegmentDist(p,a,b)

    ap = p - a;
    ab = b - a;

    t = dot(ap,ab) / (dot(ab,ab) + 1e-8);
    t = max(0,min(1,t));

    proj = a + t*ab;

    d = norm(p - proj);
end