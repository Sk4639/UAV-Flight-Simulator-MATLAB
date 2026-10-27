function uav_flight_simulator_gui
% UAV FLIGHT SIMULATOR GUI
% Interactive educational 2D UAV navigation and obstacle avoidance demo.
% Not certified flight-control software. Do not use with real aircraft.

clc;
close all;

% ---- Simulation parameters ----
startPos = [1, 1];
goalPos = [9, 9];
worldLimits = [0 10];
obstacles = [ ...
    3.2 3.5 0.75; ...
    5.3 5.5 0.95; ...
    7.1 7.2 0.70; ...
    4.3 7.4 0.65];

stepSize = 0.10;
goalTolerance = 0.20;
influenceDistance = 1.65;
maxSteps = 1600;
goalGain = 1.0;
repulsionGain = 1.25;

path = [];
currentIndex = 1;
isRunning = false;
simTimer = [];
uavMarker = [];
pathLine = [];
statusLabel = [];
metricsLabel = [];
speedSlider = [];
startButton = [];
pauseButton = [];
resetButton = [];

% ---- Create application window ----
fig = uifigure('Name','UAV Flight Simulator | Collision Avoidance', ...
    'Position',[100 80 1120 700], 'Color',[0.96 0.97 0.99], ...
    'CloseRequestFcn',@closeApp);

mainGrid = uigridlayout(fig,[1 2]);
mainGrid.ColumnWidth = {'3x','1x'};
mainGrid.RowHeight = {'1x'};
mainGrid.Padding = [12 12 12 12];
mainGrid.ColumnSpacing = 12;

ax = uiaxes(mainGrid);
ax.Layout.Row = 1;
ax.Layout.Column = 1;
ax.XLim = worldLimits;
ax.YLim = worldLimits;
ax.DataAspectRatio = [1 1 1];
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.Box = 'on';
xlabel(ax,'X position (simulation units)');
ylabel(ax,'Y position (simulation units)');
title(ax,'Interactive UAV Waypoint Navigation');

panel = uipanel(mainGrid,'Title','Flight Controls','FontWeight','bold');
panel.Layout.Row = 1;
panel.Layout.Column = 2;
controls = uigridlayout(panel,[12 1]);
controls.RowHeight = {34, 26, 42, 42, 42, 26, 30, 30, 26, 70, '1x', 25};
controls.Padding = [12 12 12 12];
controls.RowSpacing = 8;

uilabel(controls,'Text','UAV Collision Avoidance Demo', ...
    'FontSize',16,'FontWeight','bold','WordWrap','on');

statusLabel = uilabel(controls,'Text','Status: Ready', ...
    'FontSize',13,'FontWeight','bold','FontColor',[0.05 0.35 0.15]);

startButton = uibutton(controls,'push','Text','▶  Start Simulation', ...
    'ButtonPushedFcn',@startSim,'FontSize',13);
pauseButton = uibutton(controls,'push','Text','Ⅱ  Pause Simulation', ...
    'ButtonPushedFcn',@pauseSim,'FontSize',13);
resetButton = uibutton(controls,'push','Text','↺  Reset Simulation', ...
    'ButtonPushedFcn',@resetSim,'FontSize',13);

uilabel(controls,'Text','Simulation speed','FontWeight','bold');
speedSlider = uislider(controls,'Limits',[1 10],'Value',5);
speedSlider.MajorTicks = [1 3 5 7 10];

uilabel(controls,'Text','Metrics','FontWeight','bold');
metricsLabel = uilabel(controls,'Text','Path length: —\nSteps: —\nMin clearance: —', ...
    'FontSize',12,'WordWrap','on');

helpLabel = uilabel(controls,'Text', ...
    ['Green = start | Blue = goal' newline ...
     'Red circles = obstacles' newline ...
     'Orange dashed circles = safety margins' newline newline ...
     'Method: goal attraction + obstacle repulsion'], ...
    'WordWrap','on','FontSize',11);

uilabel(controls,'Text','Educational simulation only', ...
    'FontColor',[0.45 0.45 0.45],'FontSize',10);

% Timer is used to animate the vehicle without blocking the GUI.
simTimer = timer('ExecutionMode','fixedSpacing','Period',0.04, ...
    'BusyMode','drop','TimerFcn',@advanceSim);

drawEnvironment();
planPath();
drawnow;

    function drawEnvironment()
        cla(ax);
        hold(ax,'on');
        ax.XLim = worldLimits;
        ax.YLim = worldLimits;
        ax.DataAspectRatio = [1 1 1];
        ax.XGrid = 'on';
        ax.YGrid = 'on';
        ax.Box = 'on';
        th = linspace(0,2*pi,100);

        for ii = 1:size(obstacles,1)
            cx = obstacles(ii,1);
            cy = obstacles(ii,2);
            r = obstacles(ii,3);
            fill(ax,cx+r*cos(th),cy+r*sin(th),[0.91 0.32 0.23], ...
                'FaceAlpha',0.65,'EdgeColor',[0.65 0.12 0.10], ...
                'HandleVisibility','off');
            safetyR = r + 0.45;
            plot(ax,cx+safetyR*cos(th),cy+safetyR*sin(th),'--', ...
                'Color',[0.95 0.58 0.10],'LineWidth',1, ...
                'HandleVisibility','off');
        end

        plot(ax,startPos(1),startPos(2),'o','MarkerSize',10, ...
            'MarkerFaceColor',[0.1 0.7 0.2],'MarkerEdgeColor','k', ...
            'DisplayName','Start');
        plot(ax,goalPos(1),goalPos(2),'p','MarkerSize',13, ...
            'MarkerFaceColor',[0.1 0.35 0.95],'MarkerEdgeColor','k', ...
            'DisplayName','Goal');

        pathLine = plot(ax,nan,nan,'-','Color',[0.05 0.20 0.65], ...
            'LineWidth',2.2,'DisplayName','Flight path');
        uavMarker = plot(ax,startPos(1),startPos(2),'^', ...
            'MarkerSize',12,'MarkerFaceColor',[0.1 0.1 0.1], ...
            'MarkerEdgeColor','w','DisplayName','UAV');
        legend(ax,'Location','southoutside','Orientation','horizontal');
        hold(ax,'off');
    end

    function planPath()
        path = zeros(maxSteps+1,2);
        path(1,:) = startPos;
        pos = startPos;
        n = 1;

        for kk = 1:maxSteps
            if norm(goalPos-pos) <= goalTolerance
                break;
            end

            force = goalGain*(goalPos-pos);
            for jj = 1:size(obstacles,1)
                center = obstacles(jj,1:2);
                radius = obstacles(jj,3);
                delta = pos-center;
                centerDistance = norm(delta);
                surfaceDistance = max(centerDistance-radius,0.05);

                if surfaceDistance < influenceDistance
                    if centerDistance > 1e-9
                        away = delta/centerDistance;
                    else
                        away = [1 0];
                    end
                    strength = repulsionGain * ...
                        (1/surfaceDistance - 1/influenceDistance);
                    force = force + strength*away/(surfaceDistance^2);
                end
            end

            forceNorm = norm(force);
            if forceNorm > 1e-9
                direction = force/forceNorm;
            else
                direction = [0 0];
            end
            pos = pos + stepSize*direction;
            pos = min(max(pos,worldLimits(1)),worldLimits(2));
            n = n+1;
            path(n,:) = pos;
        end
        path = path(1:n,:);
        updateMetrics();
    end

    function updateMetrics()
        if isempty(path)
            return;
        end
        pathLength = sum(vecnorm(diff(path,1,1),2,2));
        clearances = zeros(size(path,1),size(obstacles,1));
        for aa = 1:size(path,1)
            for bb = 1:size(obstacles,1)
                clearances(aa,bb) = norm(path(aa,:)-obstacles(bb,1:2))-obstacles(bb,3);
            end
        end
        minClearance = min(clearances,[],'all');
        metricsLabel.Text = sprintf('Path length: %.2f units\nSteps: %d\nMin clearance: %.2f units\nGoal reached: %s', ...
            pathLength,size(path,1)-1,minClearance, ...
            string(norm(path(end,:)-goalPos)<=goalTolerance));
    end

    function startSim(~,~)
        if isRunning
            return;
        end
        if currentIndex >= size(path,1)
            resetSim();
        end
        isRunning = true;
        statusLabel.Text = 'Status: Running';
        statusLabel.FontColor = [0.05 0.35 0.15];
        start(simTimer);
    end

    function pauseSim(~,~)
        isRunning = false;
        if ~isempty(simTimer) && isvalid(simTimer) && strcmp(simTimer.Running,'on')
            stop(simTimer);
        end
        statusLabel.Text = 'Status: Paused';
        statusLabel.FontColor = [0.65 0.35 0.05];
    end

    function resetSim(~,~)
        isRunning = false;
        if ~isempty(simTimer) && isvalid(simTimer) && strcmp(simTimer.Running,'on')
            stop(simTimer);
        end
        currentIndex = 1;
        set(pathLine,'XData',nan,'YData',nan);
        set(uavMarker,'XData',startPos(1),'YData',startPos(2));
        statusLabel.Text = 'Status: Ready';
        statusLabel.FontColor = [0.05 0.35 0.15];
    end

    function advanceSim(~,~)
        if ~isRunning || currentIndex > size(path,1)
            pauseSim();
            if currentIndex >= size(path,1)
                statusLabel.Text = 'Status: Simulation complete';
            end
            return;
        end
        set(pathLine,'XData',path(1:currentIndex,1), ...
            'YData',path(1:currentIndex,2));
        set(uavMarker,'XData',path(currentIndex,1), ...
            'YData',path(currentIndex,2));
        % Keep the timer Period fixed while it is running. The speed slider
        % controls how many route points are advanced on each timer tick.
        currentIndex = currentIndex + max(1,round(speedSlider.Value/3));
        drawnow limitrate;
    end

    function closeApp(~,~)
        if ~isempty(simTimer) && isvalid(simTimer)
            stop(simTimer);
            delete(simTimer);
        end
        delete(fig);
    end
end
