function Length = SpringLength(in1,in2)
%SPRINGLENGTH
%    LENGTH = SPRINGLENGTH(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    07-Feb-2017 22:52:02

alpha = in1(4,:);
beta = in1(5,:);
l2 = in2(:,6);
l3 = in2(:,10);
lH = in2(:,3);
lL2 = in2(:,7);
lL3 = in2(:,11);
m1 = in2(:,1);
m2 = in2(:,4);
m3 = in2(:,8);
phi = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = alpha+phi;
t3 = cos(t2);
t4 = cos(phi);
t5 = lH.*t4;
t6 = lL2.*t3;
t7 = alpha+beta+phi;
t8 = cos(t7);
t10 = m1+m2+m3;
t11 = 1.0./t10;
t9 = t5+t6-y+lL3.*t8-t11.*(m3.*(t5+t6-y+l3.*t8)-m1.*y+m2.*(t5-y+l2.*t3));
t12 = sin(t2);
t13 = sin(phi);
t14 = lH.*t13;
t15 = lL2.*t12;
t16 = sin(t7);
t17 = t14+t15+x+lL3.*t16-t11.*(m1.*x+m2.*(t14+x+l2.*t12)+m3.*(t14+t15+x+l3.*t16));
Length = sqrt(t9.^2+t17.^2);