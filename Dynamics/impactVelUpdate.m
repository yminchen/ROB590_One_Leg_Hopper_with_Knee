function qplus = impactVelUpdate(x)
    param = simParameters();
    n = size(x,1);
    invM = inverseMassMatrix(x(1:5),param);
    J = JcontPoint(x(1:5),param);
    qplus = (eye(n/2) - invM*J'*inv(J*invM*J')*J)*x(n/2+1:n);
end