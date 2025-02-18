function P = Pbartransform(Pbar)
    % Transform an m-by-1 vector into an n-by-n square symmetric matrix 
    % where m and n are related by m=n*(n+1)/2.
    % This is the inverse transformation of Ptransform.

    % checks
    cond1 = ismatrix(Pbar);
    if cond1 == false
        error('The input variable is not a matrix.')
    end

    [m, p] = size(Pbar);
    cond2 = (p == 1);
    if cond2 == false
        error('The matrix is not a vector.')
    end

    n = (-1 + sqrt(1+8*m))/2;
    cond3 = (mod(n,1) == 0);
    if cond3 == false
        error("Can't convert the dimension m of Pbar into an integer n.")
    end
    n = int32(n);

    % algorithm
    i = 0;
    P = zeros(n);
    for k = 1:n
        for j = k:n
            i = i+1;
            if k ~= j
                P(k,j) = Pbar(i)/2;
                P(j,k) = P(k,j);
            else
                P(k,j) = Pbar(i);
            end
        end
    end
end