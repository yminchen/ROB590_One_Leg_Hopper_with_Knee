function Knee = posKnee(in1,in2)
%POSKNEE
%    KNEE = POSKNEE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    08-Feb-2017 11:39:18

alpha = in1(4,:);
lH = in2(:,3);
lL2 = in2(:,7);
phi = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = alpha+phi;
Knee = [x+lH.*sin(phi)+lL2.*sin(t2);y-lH.*cos(phi)-lL2.*cos(t2)];
