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
        -1, 361; ...     & Degrees with +/-1 for overshoot
        -1, 361; ...
        -1, 361]; 
    %-------------------------------
    handles = InstallThePanel(handles);
    handles = SetQlimToFinite(handles);
    handles = GetCurrentRobotState(handles);
    handles = MakeSliders(handles);
    handles = AddExitButton(handles);
    handles = AssignCallbacks(handles);
end

%% ---- VALUES ARE NOT BEING PASSED BACK FROM CALLBACK - THIS HAS UNINTENDED RESULTS WHEN JOGGING MULTIPLE AXES
function teach_callback(src, name, j, handles)
    % called on changes to a slider or to the edit box showing joint coordinate
    % src      = the object that caused the event
    % name     = name of the robot
    % j        = the index concerned (1..N)
    
    % Update the slider/edit box
    switch get(src, 'Style')
        case 'slider'
            % slider changed, get value and reflect it to edit box
            newval = get(src, 'Value');
            set(handles.edit(j), 'String', num2str(newval, 3));
        case 'edit'
            % edit box changed, get value and reflect it to slider
            newval = str2double(get(src, 'String'));
            set(handles.slider(j), 'Value', newval);
    end
    
    % Assign the relevant joint with the updated value
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
    handles.T6(modifier,sliderType) = newval;

    % find all graphical objects tagged with the robot name, this is the
    % instancs of that robot across all figures
    h = findobj('Tag', name);
    % find the graphical element of this name
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    % get the info from its Userdata
    info = get(h(1), 'UserData');
    
    % Generate new transform
    transform = transl(handles.T6(1,4),handles.T6(2,4),handles.T6(3,4)) ...
        * trotx(deg2rad(handles.T6(1,3))) ...
        * troty(deg2rad(handles.T6(2,3))) ...
        * trotz(deg2rad(handles.T6(3,3)));
    
    % Compute joint angles for new pose
    CalculateTraj(handles.robot, transform, 1)      % self, Transform, steps
    
    % Move the robot 1 step
    RunTraj(handles.robot, 1)                       % self, increment
    
    % update the stored joint coordinates
    info.q = handles.robot.qCurrent;

    % and save it back to the graphical object
    set(h(1), 'UserData', info);
    
    % Update all sliders and edit boxes
    % recompute the robot tool pose
    handles.T6 = handles.robot.model.fkine(info.q);
    
    n = size(handles.sliderLabels,2);
    for k=1:n
        switch(k)
            case {1,2,3}
                modifier=k;
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
        if(k==j)
            % We have already updated this, dont do anything
        else
            % Update all other sliders
            % reflect it to edit box
            set(handles.edit(k), 'String', num2str(handles.T6(modifier,sliderType), 3));
            % reflect it to slider
            set(handles.slider(k), 'Value', handles.T6(modifier,sliderType));
        end
    end
    


    
%     
%     % update all robots of this name
%     handles.robot.model.animate(handles.robot, info.q);
%     
%     
%     % compute the robot tool pose
%     T6 = handles.robot.fkine(info.q);
%     
%     % convert orientation to desired format
%     switch handles.orientation
%         case 'approach'
%             orient = T6(:,3);    % approach vector
%         case 'eul'
%             orient = tr2eul(T6, 'setopt', handles.opt);
%         case'rpy'
%             orient = tr2rpy(T6, 'setopt', handles.opt);
%     end
%     
%     % update the display in the teach window
%     for i=1:3
%         set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
%         set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
%     end
%     
%     if ~isempty(handles.callback)
%         handles.callback(handles.robot, info.q);
%     end
%     
%     %notify(handles.robot, 'Moved');

end

function handles = AssignCallbacks(handles)
    %---- now assign the callbacks
    n = size(handles.sliderLabels,2);
    for j=1:n
        % text edit box
        set(handles.edit(j), ...
            'Interruptible', 'off', ...
            'Callback', @(src,event)teach_callback(src, handles.robot.model.name, j, handles));
        
        % slider
        set(handles.slider(j), ...
            'Interruptible', 'off', ...
            'BusyAction', 'queue', ...
            'Callback', @(src,event)teach_callback(src, handles.robot.model.name, j, handles));
    end
end

function resize_callback(handles)
    % come here on figure resize events
    fig = gcbo;   % this figure (whose callback is executing)
    fs = get(fig, 'Position');  % get size of figure
    ps = get(handles.panel, 'Position');  % get position of the panel
    % update dimensions of the axis area
    set(handles.curax, 'Units', 'pixels', ...
        'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
    % keep the panel anchored to the top left corner
    set(handles.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
end

function quit_callback(handles)
    set(handles.fig, 'ResizeFcn', '');
    delete(handles.panel);
    set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
end

function handles = AddExitButton(handles)
    %---- add buttons
    uicontrol(handles.panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.80 handles.height*(0)+.01 0.15 handles.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.7, ...
        'CallBack', @(src,event) quit_callback(handles), ...
        'BackgroundColor', 'white', ...
        'ForegroundColor', 'red', ...
        'String', 'X');
end

function handles = MakeSliders(handles)
    %---- sliders
    % Create the sliders
    n = size(handles.sliderLabels,2);
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
    set(handles.fig, 'ResizeFcn', @(src,event) resize_callback(handles));
end
