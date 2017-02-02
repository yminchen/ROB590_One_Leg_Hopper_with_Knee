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


%%%%% Prepare figure %%%%
figure(fignum); clf; 
%%%%% Plot physical world %%%%
hold on
axis equal; 
axis([boarderL boarderR -0.1 boarderT])
axis([boarderL 2 -0.1 2])

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
% Time text
text(0.8*boarderL,0.2*boarderT,'elapsed time:','color','k');
% Animation title and label
title('One-leg Hopper Animation (2D)')
xlabel(' (m)')
ylabel(' (m)')
% Skeleton plot =============================
posB = [0 0];
posH = [0 0];
posK = [0 0];
posF = [0 0];
% Plot body link
a1 = plot([posH(1) posB(1)],[posH(2) posB(2)],'k','LineWidth',3);
% Plot hip
a2 = scatter(posH(1), posH(2),40,'MarkerEdgeColor',[0 0 0],...
          'MarkerFaceColor',[0 0 1],...
          'LineWidth',1.5);
% Plot thigh
a3 = plot([posH(1) posK(1)],[posH(2) posK(2)],'k','LineWidth',3);
% Plot knee 
a4 = scatter(posK(1), posK(2),40,'MarkerEdgeColor',[0 0 0],...
          'MarkerFaceColor',[0 0 1],...
          'LineWidth',1.5);
% Plot shin
a5 = plot([posF(1) posK(1)],[posF(2) posK(2)],'k','LineWidth',3);

% Display time on plot
a6 = text(0.8*boarderL,0.1*boarderT,['' ...
    num2str(T(1),'%1.1f') ' sec'],'color','k');

% Display states on plot
a7 = text(0.8*boarderL,0.9*boarderT,'\fontsize{10}\fontname{Arial Black}Flight','color',[0.8,0.8,0.8]);
a8 = text(0.8*boarderL,0.8*boarderT,'\fontsize{10}\fontname{Arial Black}Compression','color',[0.8,0.8,0.8]);
a9 = text(0.8*boarderL,0.7*boarderT,'\fontsize{10}\fontname{Arial Black}Thrust','color',[0.8,0.8,0.8]);
% end of skeleton plot =======================

% This for loop would show animation as well as store the animation in F().
for ti=1:length(T)            

    % plot robot
    posB = posBody(X(ti,1:5)',param);
    posH = posHip(X(ti,1:5)',param);
    posK = posKnee(X(ti,1:5)',param);
    posF = posFoot(X(ti,1:5)',param);
    
    % Plot body link
    set(a1,'XData',[posH(1) posB(1)],'YData',[posH(2) posB(2)]);
    % Plot hip
    set(a2,'XData',posH(1),'YData',posH(2));
    % Plot thigh
    set(a3,'XData',[posH(1) posK(1)],'YData',[posH(2) posK(2)]);
    % Plot knee 
    set(a4,'XData',posK(1),'YData',posK(2));
    % Plot shin
    set(a5,'XData',[posF(1) posK(1)],'YData',[posF(2) posK(2)]);
    
    % Display time on plot
    set(a6,'String',[num2str(T(ti),'%1.1f') ' sec']);
    
    % Display states on plot
    sc = S(ti);     % current state
    if sc == 1 
        set(a9,'Color',[0.8,0.8,0.8]);
        set(a7,'Color',[1,0,0]);
    elseif sc == 2 
        set(a7,'Color',[0.8,0.8,0.8]);
        set(a8,'Color',[1,0,0]);
    elseif sc == 3 
        set(a8,'Color',[0.8,0.8,0.8]);
        set(a9,'Color',[1,0,0]);
    end

    % Save frames for animaiton
    F(ti) = getframe(h,[0 0 1000 400]); 
    if F_SAVEVID
        writeVideo(vidObj,F(ti));
    end
    
end

if F_SAVEVID
    close(vidObj);
end







