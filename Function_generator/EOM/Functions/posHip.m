function Hip = posHip(in1,in2)
%POSHIP
%    HIP = POSHIP(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    29-Jan-2017 00:17:38

lH = in2(:,3);
phi = in1(3,:);
x = in1(1,:);
y = in1(2,:);
Hip = [x+lH.*sin(phi);y-lH.*cos(phi)];