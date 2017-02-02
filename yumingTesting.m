Yparam = yumingParameters();


% virtual model force
% posF = posFoot(x(1:5),param);
% posB = [x(1);x(2)];
% theta = atan((posB(1)-posF(1))/(posF(2)-posB(2)));
% L_sp = sum((posF-posB).^2)^0.5;
% k = Yparam.k;
% if phase == 3
%     k = k_des;
% end
% F(1) = -k*(Yparam.L_sp0-L_sp)*cos(theta);
% F(2) =  k*(Yparam.L_sp0-L_sp)*sin(theta);


F(1) = 0;
F(2) = 200;
F(3) = 0;
x = [0 1 0 pi/6 -pi/3, 0 0 0 0 0];
phi_f = -(x(3)+x(4)+x(5));
J = J_virtual_force(x(4),x(5),phi_f,Yparam.lH,Yparam.lL2,Yparam.lL3);
tau_kh = J*[F(2);F(3)];

disp(tau_kh);
