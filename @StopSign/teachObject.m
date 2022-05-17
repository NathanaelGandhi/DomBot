function teachObject(self)
%TEACHCARTESIAN Jog the robot using Cartesian coordinates
    %-------------------------------
    % parameters for teach panel
    self.to_h.bgcol=[103 233 98]/255;  % background color
    self.to_h.height = 0.06;  % height of slider rows
    % Slider properties
    self.to_h.sliderLabels = ['X','Y','Z','R', 'P', 'Y'];
    
    sliderTollerance = 0.05;    % 5cm tollerance
    self.to_h.sliderLimits = [... 
        -4-sliderTollerance, 4+sliderTollerance; ...    %Based on the simulated environment
        -4-sliderTollerance, 4+sliderTollerance; ...    %Based on the simulated environment
        1, 2+sliderTollerance; ...                      %Based on the simulated above the table
        -181, 181; ...     & Degrees with +/-1 for overshoot
        -181, 181; ...
        -181, 181]; 
    %-------------------------------
    InstallTheObjectPanel(self);
    MakeObjectSliders(self);
    CreateObjectPositionDisplay(self);
    CreateObjectOrientationDisplay(self);
    AddExitButtonObject(self);
    AssignObjectCallbacks(self);
end

function teach_Objectcallback(src, self, j)
    % called on changes to a slider or to the edit box showing joint coordinate
    % src      = the object that caused the event
    % name     = name of the robot
    % j        = the index concerned (1..N)
    
    % Get the updated value
    switch get(src, 'Style')
        case 'slider'
            % slider changed, get value
            newval = get(src, 'Value');
        case 'edit'
            % edit box changed, get value
            newval = str2double(get(src, 'String'));
    end
    
    % Get the angles
    angles = tr2rpy(self.pose);                        % Radians
    
    newPose = self.pose;
    
    % Assign the relevant updated value
    switch(j)
        case {1,2,3}
            % Got XYZ position just update
            newPose(j,4) = newval;
        case {4,5,6}
            % Got an angle in radians
            newval = deg2rad(newval);
            angles(j-3) = newval;              % Radians
    end
    
    % Generate new end effector transform
    t = transl(newPose(1,4),newPose(2,4),newPose(3,4));   % Translation component
    r = rpy2tr(angles);                                         % Rotation component
    tr = t*r;
    % Update the Stop Sign model
    self.UpdatePose(tr);                      % Radians
    
    % Update all sliders and edit boxes
    n = size(self.to_h.sliderLabels,2);
    for k=1:n
        angles = tr2rpy(self.pose);
        % Check if XYZ or Angle
        switch(k)
            case {1,2,3}
                % Get XYZ positions
                val = self.pose(k,4);
                % reflect it to edit box
                set(self.to_h.edit(k), 'String', num2str(val, 3));
                % reflect it to slider
                set(self.to_h.slider(k), 'Value', val);          % Degrees
            case {4,5,6}
                % Get the angles
                val = angles(k-3);  
                % reflect it to edit box
                set(self.to_h.edit(k), 'String', num2str(rad2deg(val), 3));
                % reflect it to slider
                set(self.to_h.slider(k), 'Value', rad2deg(val));          % Degrees
        end
    end
    
    % update the display in the teach window
    for i=1:3
        set(self.to_h.t6.t(i), 'String', sprintf('%.3f', self.pose(i,4)));
        set(self.to_h.t6.r(i), 'String', sprintf('%.3f', rad2deg(angles(i))));    % Degrees
    end
end

function AssignObjectCallbacks(self)
    %---- now assign the callbacks
    n = size(self.to_h.sliderLabels,2);
    for j=1:n
        % text edit box
        set(self.to_h.edit(j), ...
            'Interruptible', 'off', ...
            'Callback', @(src,event)teach_Objectcallback(src, self, j));
        
        % slider
        set(self.to_h.slider(j), ...
            'Interruptible', 'off', ...
            'BusyAction', 'queue', ...
            'Callback', @(src,event)teach_Objectcallback(src, self, j));
    end
end

function resize_Objectcallback(self)
    % come here on figure resize events
    fig = gcbo;   % this figure (whose callback is executing)
    fs = get(fig, 'Position');  % get size of figure
    ps = get(self.to_h.panel, 'Position');  % get position of the panel
    % update dimensions of the axis area
    set(self.to_h.curax, 'Units', 'pixels', ...
        'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
    % keep the panel anchored to the top left corner
    set(self.to_h.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
end

function quit_Objectcallback(self)
    set(self.to_h.fig, 'ResizeFcn', '');
    delete(self.to_h.panel);
    set(self.to_h.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
end

function AddExitButtonObject(self)
    %---- add buttons
    uicontrol(self.to_h.panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.80 self.to_h.height*(0)+.01 0.15 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.7, ...
        'CallBack', @(src,event) quit_Objectcallback(self), ...
        'BackgroundColor', 'white', ...
        'ForegroundColor', 'red', ...
        'String', 'X');
end

function CreateObjectOrientationDisplay(self)
 %---- set up the orientation display box
    % R
    uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.to_h.bgcol, ...
        'Position', [0.05 1-5*self.to_h.height 0.2 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', self.to_h.sliderLabels(4));
    
    self.to_h.t6.r(1) = uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-5*self.to_h.height 0.6 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', rad2deg(self.pose(1,3))));    % Degrees
    
    % P
    uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.to_h.bgcol, ...
        'Position', [0.05 1-6*self.to_h.height 0.2 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', self.to_h.sliderLabels(5));
    
    self.to_h.t6.r(2) = uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-6*self.to_h.height 0.6 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', rad2deg(self.pose(2,3))));    % Degrees
    
    % Y
    uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.to_h.bgcol, ...
        'Position', [0.05 1-7*self.to_h.height 0.2 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', self.to_h.sliderLabels(6));
    
    self.to_h.t6.r(3) = uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-7*self.to_h.height 0.6 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', rad2deg(self.pose(3,3))));    % Degrees 
end

function CreateObjectPositionDisplay(self)
    %---- set up the position display box 
    % X
    uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.to_h.bgcol, ...
        'Position', [0.05 1-self.to_h.height 0.2 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'x:');
    
    self.to_h.t6.t(1) = uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-self.to_h.height 0.6 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', self.pose(1,4)), ...
        'Tag', 'T6');
    
    % Y
    uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.to_h.bgcol, ...
        'Position', [0.05 1-2*self.to_h.height 0.2 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'y:');
    
    self.to_h.t6.t(2) = uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-2*self.to_h.height 0.6 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', self.pose(2,4)));
    
    % Z
    uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.to_h.bgcol, ...
        'Position', [0.05 1-3*self.to_h.height 0.2 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'z:');
    
    self.to_h.t6.t(3) = uicontrol(self.to_h.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-3*self.to_h.height 0.6 self.to_h.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', self.pose(3,4)));
end

function MakeObjectSliders(self)
    %---- sliders
    % Create the sliders
    n = size(self.to_h.sliderLabels,2);
    for j=1:n
        % slider label
        uicontrol(self.to_h.panel, 'Style', 'text', ...
            'Units', 'normalized', ...
            'BackgroundColor', self.to_h.bgcol, ...
            'Position', [0 self.to_h.height*(n-j+2) 0.15 self.to_h.height], ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.5, ...
            'String', sprintf(self.to_h.sliderLabels(:,j)));
        
        % slider itself
        switch(j)
            case {1,2,3}
                % Get XYZ positions
                val = self.pose(j,4);
            case {4,5,6}
                % Get the angles                    % Degrees
                val = rad2deg(self.pose(j-3,3));
        end
        self.to_h.slider(j) = uicontrol(self.to_h.panel, 'Style', 'slider', ...
            'Units', 'normalized', ...
            'Position', [0.15 self.to_h.height*(n-j+2) 0.55 self.to_h.height], ...
            'Min', self.to_h.sliderLimits(j,1), ...
            'Max', self.to_h.sliderLimits(j,2), ...
            'Value', val, ...
            'Tag', sprintf('Slider%d', j));
        
        % text box showing slider value, also editable
        self.to_h.edit(j) = uicontrol(self.to_h.panel, 'Style', 'edit', ...
            'Units', 'normalized', ...
            'Position', [0.70 self.to_h.height*(n-j+2)+.01 0.30 0.9*self.to_h.height], ...
            'BackgroundColor', self.to_h.bgcol, ...
            'String', num2str(val, 3), ...
            'HorizontalAlignment', 'left', ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.4, ...
            'Tag', sprintf('Edit%d', j));
    end
end

function InstallTheObjectPanel(self)
    %---- install the panel at the side of the figure
    % find the right figure to put it in
    c = findobj();      % check the current axes
    if isempty(c)
        % doesn't exist in current axes, look wider
        c = findobj(0, 'Tag', self.type);    % check all figures
        if isempty(c)
            % create robot in arbitrary pose
            self.model.plot( zeros(1, self.model.n) );
            ax = gca;
        else
            ax = get(c(1), 'Parent');               % get first axis holding the robot
        end
    else
        % found it in current axes
        ax = gca;
    end
    self.to_h.fig = get(ax, 'Parent');                % get the figure that holds the axis
    
    % shrink the current axes to make room
    %   [l b w h]
    set(ax, 'OuterPosition', [0.25 0 0.70 1]);
    
    self.to_h.curax = ax;
    
    % create the panel itself
    self.to_h.panel = uipanel(self.to_h.fig, ...
        'Title', 'Teach Object', ...
        'BackGroundColor', self.to_h.bgcol,...
        'Position', [0 0 .25 1]);
    set(self.to_h.panel, 'Units', 'pixels'); % stop automatic resizing
    set(self.to_h.fig, 'Units', 'pixels');
    set(self.to_h.fig, 'ResizeFcn', @(src,event) resize_Objectcallback(self));
end

