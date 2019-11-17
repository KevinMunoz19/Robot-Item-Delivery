function R = cuat2rot(q)
    eta = q(1);
    eps = q(2:end);
    R = (eta^2-eps'*eps)*eye(3)+2*eta*skew(eps)+2*(eps*eps');
end