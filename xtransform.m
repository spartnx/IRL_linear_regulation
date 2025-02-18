function xbar = xtransform(x)
    % Transform an n-by-1 vector x into an m-by-1 vector - where 
    % m=(n(n+1)/2 - containing the quadratic terms obtained from x

    % checks
    cond1 = ismatrix(x);
    if cond1 == false
        error('The input variable is not a matrix.')
    end

    [n, p] = size(x);
    cond2 = (p == 1);
    if cond2 == false
        error('The matrix is not a vector.')
    end

    % algorithm
    m = n*(n+1)/2;
    xbar = zeros(m,1);
    i = 0;
    for k = 1:n
        for j = k:n
            i = i+1;
            xbar(i) = x(k)*x(j);
        end
    end
end