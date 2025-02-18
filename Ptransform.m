function Pbar = Ptransform(P)
    % Transform an n-by-n square symmetric matrix into an m-by-1 vector
    % where m=n(n+1)/2

    % checks
    cond1 = ismatrix(P);
    if cond1 == false
        error('The input variable is not a matrix.')
    end

    [n, p] = size(P);
    cond2 = (n == p);
    if cond2 == false
        error('The matrix is not square.')
    end

    cond3 = issymmetric(P);
    if cond3 == false
        error('The matrix is not symmetric')
    end

    % algorithm
    m = n*(n+1)/2;
    Pbar = zeros(m,1);
    i = 0;
    for k = 1:n
        for j = k:n
            i = i+1;
            if k ~= j
                Pbar(i) = 2*P(k,j);
            else
                Pbar(i) = P(k,j);
            end
        end
    end
end