function Animation(T,X,S,tFinal,F_SAVEVID)

% settings
fignum = 1;         % figure number
param = simParameters();

% the parameters that should be passed in here in a later version of code
ter_i = 0;
target_pos = 10;

%% Preprocessing
% Calculate max min velocity
Vmax = max(X(:,6));
Vmin = min(X(:,6));

% position plot window size
h=figure(fignum); clf
set(h,'Position',[0 0 1000 400]);

%% Animation

% step through each time increment and plot results
if F_SAVEVID
    vidObj = VideoWriter('One_Leg_Hopper_with_Knee.avi');
    vidObj.FrameRate = length(T)/(tFinal-0);
    open(vidObj);
    F(length(T)).cdata = []; F(length(T)).colormap = []; % preallocate
end

boarderR = max(X(:,1))+1;
boarderL = min(X(:,1))-2;
boarderT = max(X(:,2))+0.5;
% This for loop would show animation as well as store the animation in F().
for ti=1:length(T)            

    %%%%% Prepare figure %%%%
    figure(fignum); clf; 

    %%%%% Plot physical world (1st subfigure) %%%%
    hold on
    axis equal; axis([boarderL boarderR -0.1 boarderT])

    % Plot ground
    edge = Terrain_edge(ter_i);
    groundL = floor(boarderL);
    groundR = ceil(boarderR);
    for i = 1:size(edge,1)+1
        if i == 1
            fill([groundL edge(i,1) edge(i,1) groundL],[Terrain(groundL,ter_i) Terrain(edge(i,1),ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
        elseif i == size(edge,1)+1
            fill([edge(i-1,1) groundR groundR edge(i-1,1)],[Terrain(edge(i-1,1),ter_i) Terrain(groundR,ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
        else
            fill([edge(i-1,1) edge(i,1) edge(i,1) edge(i-1,1)],[Terrain(edge(i-1,1),ter_i) Terrain(edge(i,1),ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
        end
    end
    % Plot target position
    scatter(target_pos,Terrain(target_pos,ter_i),50,'MarkerEdgeColor','b',...
              'MarkerFaceColor','g',...
              'LineWidth',1);

    % plot robot
    posB = posBody(X(ti,1:5)',param);
    posH = posHip(X(ti,1:5)',param);
    posK = posKnee(X(ti,1:5)',param);
    posF = posFoot(X(ti,1:5)',param);
    
    % Plot body link
    plot([posH(1) posB(1)],[posH(2) posB(2)],'k','LineWidth',3);
    % Plot hip
    scatter(posH(1), posH(2),40,'MarkerEdgeColor',[0 0 0],...
              'MarkerFaceColor',[0 0 1],...
              'LineWidth',1.5);
    % Plot thigh
    plot([posH(1) posK(1)],[posH(2) posK(2)],'k','LineWidth',3);
    % Plot knee 
    scatter(posK(1), posK(2),40,'MarkerEdgeColor',[0 0 0],...
              'MarkerFaceColor',[0 0 1],...
              'LineWidth',1.5);
    % Plot shin
    plot([posF(1) posK(1)],[posF(2) posK(2)],'k','LineWidth',3);
    
    % Display time on plot
    tc = T(ti);     % current time
    text(0.8*boarderL,0.2*boarderT,'elapsed time:','color','k')
    text(0.8*boarderL,0.1*boarderT,['' ...
        num2str(tc,'%1.1f') ' sec'],'color','k')
%     % Display states on plot
%     sc = S(ti);     % current state
%     text(0.8*boarderL,0.9*boarderT,'\fontsize{10}\fontname{Arial Black}Flight','color',[0.8,0.8,0.8])
%     text(0.8*boarderL,0.8*boarderT,'\fontsize{10}\fontname{Arial Black}Compression','color',[0.8,0.8,0.8])
%     text(0.8*boarderL,0.7*boarderT,'\fontsize{10}\fontname{Arial Black}Thrust','color',[0.8,0.8,0.8])
%     if sc == 1 
%         text(0.8*boarderL,0.9*boarderT,'\fontsize{10}\fontname{Arial Black}Flight','color','r')
%     elseif sc == 2 
%         text(0.8*boarderL,0.8*boarderT,'\fontsize{10}\fontname{Arial Black}Compression','color','r')
%     elseif sc == 3 
%         text(0.8*boarderL,0.7*boarderT,'\fontsize{10}\fontname{Arial Black}Thrust','color','r')
%     elseif sc == 4
%         text(0.8*boarderL,0.9*boarderT,'\fontsize{10}\fontname{Arial Black}Flight','color','b')
%     elseif sc == 5
%         text(0.8*boarderL,0.8*boarderT,'\fontsize{10}\fontname{Arial Black}Compression','color','b')
%     elseif sc == 6
%         text(0.8*boarderL,0.7*boarderT,'\fontsize{10}\fontname{Arial Black}Thrust','color','b')
%     end
%     if sc <= 3
%         text(0.7*boarderL,0.6*boarderT,'\fontsize{10}\fontname{Arial Black}R','color','r')
%         text(0.8*boarderL,0.6*boarderT,'\fontsize{10}\fontname{Arial Black}L','color',[0.8,0.8,0.8])
%     else
%         text(0.8*boarderL,0.6*boarderT,'\fontsize{10}\fontname{Arial Black}L','color','b')
%         text(0.7*boarderL,0.6*boarderT,'\fontsize{10}\fontname{Arial Black}R','color',[0.8,0.8,0.8])
%     end

    title('One-leg Hopper Animation (2D)')
    xlabel(' (m)')
    ylabel(' (m)')

    h=figure(fignum);
    F(ti) = getframe(h,[0 0 1000 400]); 
    if F_SAVEVID
        writeVideo(vidObj,F(ti));
    end

end

if F_SAVEVID
    close(vidObj);
end







