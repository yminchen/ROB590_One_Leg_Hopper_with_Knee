function dJFoot = dJcontPoint(in1,in2)
%DJCONTPOINT
%    DJFOOT = DJCONTPOINT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    29-Jan-2017 00:17:37

alpha = in1(4,:);
beta = in1(5,:);
lH = in2(:,3);
lL2 = in2(:,7);
lL3 = in2(:,11);
phi = in1(3,:);
valpha = in1(9,:);
vbeta = in1(10,:);
vphi = in1(8,:);
t2 = alpha+phi;
t3 = sin(t2);
t4 = lL2.*t3;
t5 = alpha+beta+phi;
t6 = sin(t5);
t7 = lL3.*t6;
t8 = t4+t7;
t9 = cos(t2);
t10 = lL2.*t9;
t11 = cos(t5);
t12 = lL3.*t11;
t13 = t10+t12;
t14 = t13.*valpha;
t15 = lL3.*t11.*vbeta;
t16 = valpha+vbeta+vphi;
dJFoot = reshape([0.0,0.0,0.0,0.0,-t8.*valpha-vphi.*(t4+t7+lH.*sin(phi))-lL3.*t6.*vbeta,t14+t15+vphi.*(t10+t12+lH.*cos(phi)),-t8.*valpha-t8.*vphi-lL3.*t6.*vbeta,t14+t15+t13.*vphi,-lL3.*t6.*t16,lL3.*t11.*t16],[2,5]);
