set(0,'defaulttextinterpreter','latex')
figHandle(1) = figure;
plot(T,S(:,1)); grid on; xlabel('Time'); ylabel('$X$');
figHandle(2) = figure;
plot(T,S(:,2),T,DS); grid on; xlabel('Time'); ylabel('$Y$');
figHandle(3) = figure;
plot(T,S(:,3)); grid on; xlabel('Time'); ylabel('$\phi$');
figHandle(4) = figure;
plot(T,S(:,4)); grid on; xlabel('Time'); ylabel('$\alpha$');
figHandle(5) = figure;
plot(T,S(:,5)); grid on; xlabel('Time'); ylabel('$\beta$');
figHandle(6) = figure;
plot(T,S(:,6)); grid on; xlabel('Time'); ylabel('$\dot{X}$');
figHandle(7) = figure;
plot(T,S(:,7)); grid on; xlabel('Time'); ylabel('$\dot{Y}$');
figHandle(8) = figure;
plot(T,S(:,8)); grid on; xlabel('Time'); ylabel('$\dot{\phi}$');
figHandle(9) = figure;
plot(T,S(:,9)); grid on; xlabel('Time'); ylabel('$\dot{\alpha}$');
figHandle(10) = figure;
plot(T,S(:,10)); grid on; xlabel('Time'); ylabel('$\dot{\beta}$');

if F_SAVEPLOT
    if ~exist('Data','dir')
        mkdir('Data');
    end
    cl = round(clock);
    clt = '';
    for i=1:1:6
        clt= strcat(clt,num2str(cl(i),'%02.0f'));
    end
    clt = strcat('Data\Resp',clt);
    savefig(figHandle,clt)
%     figure(figHandle(1));
%     print(strcat(clt,'\X'),'-depsc','-tiff');
%     figure(figHandle(2));
%     print(strcat(clt,'\Y'),'-depsc','-tiff');
%     figure(figHandle(3));
%     print(strcat(clt,'\Phi'),'-depsc','-tiff');
%     figure(figHandle(4));
%     print(strcat(clt,'\Alpha'),'-depsc','-tiff');
%     figure(figHandle(5));
%     print(strcat(clt,'\Beta'),'-depsc','-tiff');
%     figure(figHandle(6));
%     print(strcat(clt,'\VX'),'-depsc','-tiff');
%     figure(figHandle(7));
%     print(strcat(clt,'\VY'),'-depsc','-tiff');
%     figure(figHandle(8));
%     print(strcat(clt,'\VPhi'),'-depsc','-tiff');
%     figure(figHandle(9));
%     print(strcat(clt,'\VAlpha'),'-depsc','-tiff');
%     figure(figHandle(10));
%     print(strcat(clt,'\VBeta'),'-depsc','-tiff');
end
if ~F_PLOT
    close(figHandle)
end