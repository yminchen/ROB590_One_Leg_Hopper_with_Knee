function J_reduced = J_virtual_force(alpha,beta,phi_f,lH,lL2,lL3)
%J_VIRTUAL_FORCE
%    J_REDUCED = J_VIRTUAL_FORCE(ALPHA,BETA,PHI_F,LH,LL2,LL3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    31-Jan-2017 17:20:27

t2 = alpha+beta+phi_f;
t3 = cos(t2);
t4 = 1.0./t3;
t5 = sin(alpha);
t6 = 1.0./lH;
t7 = beta+phi_f;
t8 = cos(t7);
J_reduced = reshape([lL2.*t4.*t5,t4.*(lL2.*t5+lL3.*sin(alpha+beta)),lL2.*t4.*t6.*t8,t4.*t6.*(lL2.*t8+lL3.*cos(phi_f))],[2,2]);