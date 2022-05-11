function teachCartesian(robot)
%TEACHCARTESIAN Jog the robot using Cartesian coordinates
    %-------------------------------
    % parameters for teach panel
    handles.robot=robot;
    handles.bgcol=[103 233 98]/255;  % background color
    handles.height = 0.06;  % height of slider rows
    % Slider properties
    handles.sliderLabels = ['X','Y','Z',char(hex2dec('3c6')),char(hex2dec('3b8')),char(hex2dec('3c8'))];
    handles.sliderLimits = [... 
        -handles.robot.radiusOfMotion, handles.robot.radiusOfMotion; ...
        -handles.robot.radiusOfMotion, handles.robot.radiusOfMotion; ...
        handles.robot.pose(3,4), handles.robot.pose(3,4)+0.412; ...         %0.411 appears to be max simulated reach
        0, 2*pi; ...
        0, 2*pi; ...
        0, 2*pi;]; 
    %-------------------------------
    handles = InstallThePanel(handles);
    handles = SetQlimToFinite(handles);
    handles = GetCurrentRobotState(handles);
    MakeSliders(handles);
    AddExitButton(handles);
end

function AddExitButton(handles)
    %---- add buttons
    uicontrol(handles.panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.80 handles.height*(0)+.01 0.15 handles.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.7, ...
        'CallBack', @(src,event) quit_callback(handles.robot, handles), ...
        'BackgroundColor', 'white', ...
        'ForegroundColor', 'red', ...
        'String', 'X');
end

function MakeSliders(handles)
    %---- sliders
    % Create the sliders
    n = size(handles.sliderLabels,2);      % xyz
    for j=1:n
        % slider label
        uicontrol(handles.panel, 'Style', 'text', ...
            'Units', 'normalized', ...
            'BackgroundColor', handles.bgcol, ...
            'Position', [0 handles.height*(n-j+2) 0.15 handles.height], ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.5, ...
            'String', sprintf(handles.sliderLabels(:,j)));
        
        % slider itself
        switch(j)
            case {1,2,3}
                modifier=j;
                sliderType = 4;
            case 4
                modifier=1;
                sliderType = 3;
            case 5
                modifier=2;
                sliderType = 3;
            case 6
                modifier=3;
                sliderType = 3;
        end
        handles.slider(j) = uicontrol(handles.panel, 'Style', 'slider', ...
            'Units', 'normalized', ...
            'Position', [0.15 handles.height*(n-j+2) 0.65 handles.height], ...
            'Min', handles.sliderLimits(j,1), ...
            'Max', handles.sliderLimits(j,2), ...
            'Value', handles.T6(modifier,sliderType), ...
            'Tag', sprintf('Slider%d', j));
        
        % text box showing slider value, also editable
        handles.edit(j) = uicontrol(handles.panel, 'Style', 'edit', ...
            'Units', 'normalized', ...
            'Position', [0.80 handles.height*(n-j+2)+.01 0.20 0.9*handles.height], ...
            'BackgroundColor', handles.bgcol, ...
            'String', num2str(handles.T6(modifier,sliderType), 3), ...
            'HorizontalAlignment', 'left', ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.4, ...
            'Tag', sprintf('Edit%d', j));
    end
end

function handles = GetCurrentRobotState(handles)
    %---- get the current robot state
    if isempty(handles.q)
        % check to see if there are any graphical robots of this name
        rhandles = findobj('Tag', handles.robot.model.name);
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
    handles.robot.model.plot(handles.q);
    end
    handles.T6 = handles.robot.model.fkine(handles.q);
end

function handles = SetQlimToFinite(handles)
    % we need to have qlim set to finite values for a prismatic joint
    qlim = handles.robot.model.qlim;
    if any(isinf(qlim))
        error('RTB:teach:badarg', 'Must define joint coordinate limits for prismatic axes, set qlim properties for prismatic Links');
    end
    handles.q = [];
end

function handles = InstallThePanel(handles)
    %---- install the panel at the side of the figure
    % find the right figure to put it in
    c = findobj(gca, 'Tag', handles.robot.model.name);      % check the current axes
    if isempty(c)
        % doesn't exist in current axes, look wider
        c = findobj(0, 'Tag', handles.robot.model.name);    % check all figures
        if isempty(c)
            % create robot in arbitrary pose
            handles.robot.model.plot( zeros(1, handles.robot.model.n) );
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
        'BackGroundColor', handles.bgcol,...
        'Position', [0 0 .25 1]);
    set(panel, 'Units', 'pixels'); % stop automatic resizing
    handles.panel = panel;
    set(handles.fig, 'Units', 'pixels');
    set(handles.fig, 'ResizeFcn', @(src,event) resize_callback(handles.robot.model, handles));
end

