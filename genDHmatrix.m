function A = genDHmatrix(theta, d, a, alpha)

    Rz = [cos(theta),-sin(theta),0,0; sin(theta),cos(theta),0,0; 0,0,1,0; 0,0,0,1];
    Tz = [1,0,0,0; 0,1,0,0; 0,0,1,d; 0,0,0,1];
    Tx = [1,0,0,a; 0,1,0,0; 0,0,1,0; 0,0,0,1];
    Rx = [1,0,0,0; 0,cos(alpha),-sin(alpha),0; 0,sin(alpha),cos(alpha),0; 0,0,0,1];
    A = Rz*Tz*Tx*Rx;      
end