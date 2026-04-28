 function bad = edgeTooCloseToPoint(p1,p2,pt,epsilon)

    d = pointToSegmentDist(pt,p1,p2);
    bad = (d < epsilon);
end