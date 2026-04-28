function q = haltonSample(i, boundary)

    boundary = boundary(:)';
    xmin = boundary(1);
    ymin = boundary(2);
    xmax = boundary(3);
    ymax = boundary(4);

    qx = halton(i, 2);
    qy = halton(i, 3);

    q = [xmin + qx*(xmax - xmin), ...
         ymin + qy*(ymax - ymin)];
end

function h = halton(i, base)
    f = 1;
    r = 0;

    while i > 0
        f = f / base;
        r = r + f * mod(i, base);
        i = floor(i / base);
    end

    h = r;
end