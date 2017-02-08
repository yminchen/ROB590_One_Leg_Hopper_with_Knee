function CoG3 = CoGShank(in1,in2)
%COGSHANK
%    COG3 = COGSHANK(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    08-Feb-2017 11:39:18

alpha = in1(4,:);
beta = in1(5,:);
l3 = in2(:,10);
lH = in2(:,3);
lL2 = in2(:,7);
phi = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = alpha+phi;
t3 = alpha+beta+phi;
CoG3 = [x+lH.*sin(phi)+l3.*sin(t3)+lL2.*sin(t2);y-lH.*cos(phi)-l3.*cos(t3)-lL2.*cos(t2)];