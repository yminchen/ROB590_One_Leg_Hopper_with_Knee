
% plot states

if plot_flag(1)
    hold on
    plot(T,P(:,1),'b','DisplayName','x')
    hold off 
end
if plot_flag(2)
    hold on
    plot(T,P(:,2),'r','DisplayName','y')
    hold off 
end
if plot_flag(3)
    hold on
    plot(T,P(:,3),'g','DisplayName','phi')
    hold off 
end
if plot_flag(4)
    hold on
    plot(T,P(:,4),'k','DisplayName','alpha')
    hold off 
end
if plot_flag(5)
    hold on
    plot(T,P(:,5),'m','DisplayName','beta')
    hold off 
end
if plot_flag(6)
    hold on
    plot(T,P(:,6),'b--','DisplayName','xDot')
    hold off 
end
if plot_flag(7)
    hold on
    plot(T,P(:,7),'r--','DisplayName','yDot')
    hold off 
end
if plot_flag(8)
    hold on
    plot(T,P(:,8),'g--','DisplayName','phiDot')
    hold off 
end
if plot_flag(9)
    hold on
    plot(T,P(:,9),'k--','DisplayName','alphaDot')
    hold off 
end
if plot_flag(10)
    hold on
    plot(T,P(:,10),'m--','DisplayName','betaDot')
    hold off 
end
if plot_flag(11)
    hold on
    plot(T,P(:,11),'Color',[0.5 0 0],'DisplayName','L')
    hold off 
end
if plot_flag(12)
    hold on
    plot(T,P(:,12),'--','Color',[0.5 0 0],'DisplayName','LDot')
    hold off 
end
if plot_flag(13)
    hold on
    plot(T,P(:,13),'--','Color',[0.8 0 0],'DisplayName','Energy')
    hold off 
end
if plot_flag(14)
    hold on
    plot(T,P(:,14),'--','Color',[0.8 0 0],'DisplayName','tau_{hip}')
    hold off 
end
if plot_flag(15)
    hold on
    plot(T,P(:,15),'--','Color',[0 0 0.8],'DisplayName','tau_{knee}')
    hold off 
end
if plot_flag(16)
    hold on
    plot(T,P(:,16),'Color',[1 0 0],'DisplayName','F_cx')
    hold off 
end
if plot_flag(17)
    hold on
    plot(T,P(:,17),'Color',[0 0 1],'DisplayName','F_cy')
    hold off 
end

