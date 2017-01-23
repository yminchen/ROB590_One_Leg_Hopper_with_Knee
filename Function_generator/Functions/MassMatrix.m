function M = MassMatrix(in1,in2)
%MASSMATRIX
%    M = MASSMATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    23-Jan-2017 16:59:40

J1 = in2(:,2);
J2 = in2(:,5);
alpha = in1(4,:);
beta = in1(5,:);
l3 = in2(:,10);
lH = in2(:,3);
lL2 = in2(:,7);
m1 = in2(:,1);
m2 = in2(:,4);
m3 = in2(:,8);
phi = in1(3,:);
t2 = m2+m3;
t3 = alpha+phi;
t4 = cos(t3);
t5 = lL2.*t4;
t6 = alpha+beta+phi;
t7 = cos(t6);
t8 = l3.*t7;
t9 = m1+m2+m3;
t10 = sin(t3);
t11 = lL2.*t10;
t12 = sin(t6);
t13 = l3.*t12;
t14 = cos(phi);
t15 = lH.*t14;
t16 = t5+t8+t15;
t17 = t2.*t16;
t18 = sin(phi);
t19 = lH.*t18;
t20 = t11+t13+t19;
t21 = t2.*t20;
t22 = l3.^2;
t23 = lH.^2;
t24 = lL2.^2;
t25 = alpha+beta;
t26 = cos(t25);
t27 = cos(alpha);
t28 = cos(beta);
t29 = m2.*t22;
t30 = m3.*t22;
t31 = m2.*t24;
t32 = m3.*t24;
t33 = l3.*lL2.*m2.*t28.*2.0;
t34 = l3.*lL2.*m3.*t28.*2.0;
t35 = t5+t8;
t36 = t2.*t35;
t37 = t11+t13;
t38 = t2.*t37;
t39 = l3.*lH.*m2.*t26;
t40 = l3.*lH.*m3.*t26;
t41 = lH.*lL2.*m2.*t27;
t42 = lH.*lL2.*m3.*t27;
t43 = J2+t29+t30+t31+t32+t33+t34+t39+t40+t41+t42;
t44 = lL2.*t28;
t45 = l3.*t2.*t7;
t46 = l3.*t2.*t12;
t47 = lH.*t26;
t48 = l3+t44+t47;
t49 = l3.*t2.*t48;
t50 = l3+t44;
t51 = l3.*t2.*t50;
M = reshape([t9,0.0,t17,t36,t45,0.0,t9,t21,t38,t46,t17,t21,J1+J2+t29+t30+t31+t32+t33+t34+m2.*t23+m3.*t23+l3.*lH.*m2.*t26.*2.0+l3.*lH.*m3.*t26.*2.0+lH.*lL2.*m2.*t27.*2.0+lH.*lL2.*m3.*t27.*2.0,t43,t49,t36,t38,t43,J2+t29+t30+t31+t32+t33+t34,t51,t45,t46,t49,t51,t2.*t22],[5,5]);
