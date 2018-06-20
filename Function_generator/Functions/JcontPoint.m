function JFoot = JcontPoint(in1,in2)
%JCONTPOINT
%    JFOOT = JCONTPOINT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    25-Jan-2017 10:33:40

alpha = in1(4,:);
beta = in1(5,:);
lH = in2(:,3);
lL2 = in2(:,7);
lL3 = in2(:,11);
phi = in1(3,:);
t2 = alpha+phi;
t3 = cos(t2);
t4 = lL2.*t3;
t5 = alpha+beta+phi;
t6 = cos(t5);
t7 = lL3.*t6;
t8 = sin(t2);
t9 = lL2.*t8;
t10 = sin(t5);
t11 = lL3.*t10;
JFoot = reshape([1.0,0.0,0.0,1.0,t4+t7+lH.*cos(phi),t9+t11+lH.*sin(phi),t4+t7,t9+t11,t7,t11],[2,5]);