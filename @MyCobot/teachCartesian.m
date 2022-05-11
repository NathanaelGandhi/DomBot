function teachCartesian(robot)
%TEACHCARTESIAN Jog the robot using Cartesian coordinates
    %-------------------------------
    % parameters for teach panel
    bgcol = [103 233 98]/255;  % background color
    height = 0.06;  % height of slider rows
    %-------------------------------
    handles = InstallThePanel(robot,bgcol);
    handles = SetQlimToFinite(robot,handles);
    handles = GetCurrentRobotState(robot,handles);
    MakeSliders(robot,handles,bgcol,height);
end

function MakeSliders(robot,handles,bgcol,height)
    %---- sliders
    % Slider properties
    sliderLabels = ['x','y','z'];
    sliderLimits = [... 
        -robot.radiusOfMotion, robot.radiusOfMotion; ...
        -robot.radiusOfMotion, robot.radiusOfMotion; ...
        robot.pose(3,4), robot.pose(3,4)+0.412]; %0.411 appears to be max simulated reach
    % Create the sliders
    n = size(sliderLabels,2);      % xyz
    for j=1:n
        % slider label
        uicontrol(handles.panel, 'Style', 'text', ...
            'Units', 'normalized', ...
            'BackgroundColor', bgcol, ...
            'Position', [0 height*(n-j+2) 0.15 height], ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.5, ...
            'String', sprintf(sliderLabels(:,j)));
        
        % slider itself
        handles.slider(j) = uicontrol(handles.panel, 'Style', 'slider', ...
            'Units', 'normalized', ...
            'Position', [0.15 height*(n-j+2) 0.65 height], ...
            'Min', sliderLimits(j,1), ...
            'Max', sliderLimits(j,2), ...
            'Value', handles.T6(j,4), ...
            'Tag', sprintf('Slider%d', j));
%         
%         % text box showing slider value, also editable
%         handles.edit(j) = uicontrol(panel, 'Style', 'edit', ...
%             'Units', 'normalized', ...
%             'Position', [0.80 height*(n-j+2)+.01 0.20 0.9*height], ...
%             'BackgroundColor', bgcol, ...
%             'String', num2str(qscale(j)*q(j), 3), ...
%             'HorizontalAlignment', 'left', ...
%             'FontUnits', 'normalized', ...
%             'FontSize', 0.4, ...
%             'Tag', sprintf('Edit%d', j));
    end
end

function handles = GetCurrentRobotState(robot, handles)
    %---- get the current robot state
    if isempty(handles.q)
        % check to see if there are any graphical robots of this name
        rhandles = findobj('Tag', robot.model.name);
        % find the graphical element of this name
        if isempty(rhandles)
            error('RTB:teach:badarg', 'No graphical robot of this name found');
        end
        % get the info from its Userdata
        info = get(rhandles(1), 'UserData');
        % the handle contains current joint angles (set by plot)
        if ~isempty(info.q)
            handles.q = info.q;
        end
    else
    robot.model.plot(handles.q);
    end
    handles.T6 = robot.model.fkine(handles.q);
end

function handles = SetQlimToFinite(robot,handles)
    % we need to have qlim set to finite values for a prismatic joint
    qlim = robot.model.qlim;
    if any(isinf(qlim))
        error('RTB:teach:badarg', 'Must define joint coordinate limits for prismatic axes, set qlim properties for prismatic Links');
    end
    handles.q = [];
end

function handles = InstallThePanel(robot,bgcol)
    %---- install the panel at the side of the figure
    % find the right figure to put it in
    c = findobj(gca, 'Tag', robot.model.name);      % check the current axes
    if isempty(c)
        % doesn't exist in current axes, look wider
        c = findobj(0, 'Tag', robot.model.name);    % check all figures
        if isempty(c)
            % create robot in arbitrary pose
            robot.model.plot( zeros(1, robot.model.n) );
            ax = gca;
        else
            ax = get(c(1), 'Parent');               % get first axis holding the robot
        end
    else
        % found it in current axes
        ax = gca;
    end
    handles.fig = get(ax, 'Parent');                % get the figure that holds the axis
    
    % shrink the current axes to make room
    %   [l b w h]
    set(ax, 'OuterPosition', [0.25 0 0.70 1])
    
    handles.curax = ax;
    
    % create the panel itself
    panel = uipanel(handles.fig, ...
        'Title', 'Teach Cartesian', ...
        'BackGroundColor', bgcol,...
        'Position', [0 0 .25 1]);
    set(panel, 'Units', 'pixels'); % stop automatic resizing
    handles.panel = panel;
    set(handles.fig, 'Units', 'pixels');
    set(handles.fig, 'ResizeFcn', @(src,event) resize_callback(robot.model, handles));
end

