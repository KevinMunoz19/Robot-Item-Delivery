function K = getFK(DH)
    K = eye(4);
    for i = 1:size(DH, 1)
        A = genDHmatrix(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
        K = K * A; 
    end
end