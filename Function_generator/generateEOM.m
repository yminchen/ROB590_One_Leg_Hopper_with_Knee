%% This code generates the equations of motion for a kneed hopper
%   Author: Apoorv Shrivastava
%   Output: 
%           MassMatrix  in1:[x y phi alpha beta] 
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]   
%           inverseMassMatrix
%                       in1:[x y phi alpha beta] 
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]   
%           FCorGrav    
%                       in1:[x y phi alpha beta vx vy vphi valpha vbeta]
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]   
%           posFoot
%                       in1:[x y phi alpha beta] 
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]
%           posHip
%                       in1:[x y phi alpha beta] 
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]
%           posKnee
%                       in1:[x y phi alpha beta] 
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]
%           posBody
%                       in1:[x y phi alpha beta] 
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]
%           JcontPoint
%                       in1:[x y phi alpha beta]
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]
%           dJcontPoint
%                       in1:[x y phi alpha beta vx vy vphi valpha vbeta]
%                       in2:[m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g]
%
%% Some Housekeeping 
clear; clc; 
%% State Variable declarations
syms x y phi alpha beta vx vy vphi valpha vbeta
q = [x; y; phi; alpha; beta];
vq = [vx; vy; vphi; valpha; vbeta];
%% Parameters
% System
syms m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3
% Environmental 
syms g
param = [m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g];
%% Calculations
%Mass & Joint position calculations
CoG1 = [x;y];
Hip = CoG1 + [lH*sin(phi);
    - lH*cos(phi)];
CoG2 = Hip + [l2*sin(phi+alpha);
    -l2*cos(phi+alpha)];
Knee = Hip + [lL2*sin(phi+alpha);
    -lL2*cos(phi+alpha)];
CoG3 = Knee + [l3*sin(phi+alpha+beta);
    - l3*cos(phi+alpha+beta)];
Foot = Knee + [lL3*sin(phi+alpha+beta);
    - lL3*cos(phi+alpha+beta)];

% Mass velocity calculations
vCoG1 = jacobian(CoG1,q)*vq;
vCoG2 = jacobian(CoG2,q)*vq;
vCoG3 = jacobian(CoG3,q)*vq;

% Lagrangian Calculations
T = 0.5*( m1*sum(vCoG1.^2) + J1*vphi^2 + m2*sum(vCoG2.^2) + J2*(vphi+valpha)^2 + m3*sum(vCoG3.^2) + J3*(vphi+valpha+vbeta)^2);
U = m1*CoG1(2)*g + m2*CoG2(2)*g + m3*CoG3(2)*g;
L = simplify(T-U);

dL_dq = jacobian(L,q).';
dL_dqdt = jacobian(L,vq).';
ddL_dqdt_dt = jacobian(dL_dqdt,q)*vq;

% Getting EOM terms
M = jacobian(dL_dqdt,vq).';
M = simplify(M);
invM = simplify(inv(M));

fCG = simplify(dL_dq - ddL_dqdt_dt);

% Calculating contact point jacobian
JFoot = jacobian(Foot,q);
JFoot = simplify(JFoot);
dJFoot = sym(zeros(size(JFoot)));
for i = 1:size(JFoot,2)
    dJFoot(:,i) = jacobian(JFoot(:,i),q)*vq;
end
dJFoot = simplify(dJFoot);

%% Create functions
if ~exist('Functions','dir')
    mkdir('Functions');
end

matlabFunction(M,'file','Functions\MassMatrix','vars',{q,param});
matlabFunction(invM,'file','Functions\inverseMassMatrix','vars',{q,param});
matlabFunction(fCG,'file','Functions\FCorGrav','vars',{[q;vq],param});
matlabFunction(Foot,'file','Functions\posFoot','vars',{q,param});
matlabFunction(JFoot,'file','Functions\JcontPoint','vars',{q,param});
matlabFunction(dJFoot,'file','Functions\dJcontPoint','vars',{[q;vq],param});
matlabFunction(Hip,'file','Functions\posHip','vars',{q,param});
matlabFunction(Knee,'file','Functions\posKnee','vars',{q,param});
matlabFunction(CoG1,'file','Functions\posBody','vars',{q,param});