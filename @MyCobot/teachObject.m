function teachObject(self)
%TEACHCARTESIAN Jog the robot using Cartesian coordinates
    %-------------------------------
    % parameters for teach panel
    self.tc.bgcol=[103 233 98]/255;  % background color
    self.tc.height = 0.06;  % height of slider rows
    % Slider properties
    self.tc.sliderLabels = ['X','Y','Z','R', 'P', 'Y'];
    
    sliderTollerance = 0.05;    % 5cm tollerance
    self.tc.sliderLimits = [... 
        -self.RADIUS_REACH-sliderTollerance, self.RADIUS_REACH+sliderTollerance; ...
        -self.RADIUS_REACH-sliderTollerance, self.RADIUS_REACH+sliderTollerance; ...
        self.pose(3,4), self.pose(3,4)+0.411+sliderTollerance; ...  %0.411 appears to be max simulated reach
        -181, 181; ...     & Degrees with +/-1 for overshoot
        -181, 181; ...
        -181, 181]; 
    %-------------------------------
    InstallTheObjectPanel(self);
    GetObjectTransform(self);
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
    steps = 1;
    
    % Get the updated value
    switch get(src, 'Style')
        case 'slider'
            % slider changed, get value
            newval = get(src, 'Value');
        case 'edit'
            % edit box changed, get value
            newval = str2double(get(src, 'String'));
    end
    
    GetObjectTransform(self);
    
    % Get the angles
    angles = tr2rpy(self.tc.T6);                        % Radians
    
    % Assign the relevant updated value
    switch(j)
        case {1,2,3}
            % Got XYZ position just update
            self.tc.T6(j,4) = newval;
        case {4,5,6}
            % Got an angle in radians
            newval = deg2rad(newval);
            angles(j-3) = newval;              % Radians
    end
    
    % Generate new end effector transform
    self.tc.T6 = transl(self.tc.T6(1,4),self.tc.T6(2,4),self.tc.T6(3,4)) ...
        * rpy2tr(angles);                     % Radians
   
    % Compute joint angles for new pose
    CalculateTraj(self, self.tc.T6, steps)      % self, Transform, steps
    %self.qMatrix = self.model.ikcon(self.tc.T6, self.qCurrent);
    for i=1:steps
        % Move the robot 1 step
        RunTraj(self)                           % self, increment
    end
    
    % Update (recompute) the robot tool pose
    GetObjectTransform(self);
    
    % Update all sliders and edit boxes
    n = size(self.tc.sliderLabels,2);
    for k=1:n
        angles = tr2rpy(self.tc.T6);
        % Check if XYZ or Angle
        switch(k)
            case {1,2,3}
                % Get XYZ positions
                val = self.tc.T6(k,4);
                % reflect it to edit box
                set(self.tc.edit(k), 'String', num2str(val, 3));
                % reflect it to slider
                set(self.tc.slider(k), 'Value', val);          % Degrees
            case {4,5,6}
                % Get the angles
                val = angles(k-3);  
                % reflect it to edit box
                set(self.tc.edit(k), 'String', num2str(rad2deg(val), 3));
                % reflect it to slider
                set(self.tc.slider(k), 'Value', rad2deg(val));          % Degrees
        end
    end
    
    % update the display in the teach window
    for i=1:3
        set(self.tc.t6.t(i), 'String', sprintf('%.3f', self.tc.T6(i,4)));
        set(self.tc.t6.r(i), 'String', sprintf('%.3f', rad2deg(angles(i))));    % Degrees
    end
end

function AssignObjectCallbacks(self)
    %---- now assign the callbacks
    n = size(self.tc.sliderLabels,2);
    for j=1:n
        % text edit box
        set(self.tc.edit(j), ...
            'Interruptible', 'off', ...
            'Callback', @(src,event)teach_Objectcallback(src, self, j));
        
        % slider
        set(self.tc.slider(j), ...
            'Interruptible', 'off', ...
            'BusyAction', 'queue', ...
            'Callback', @(src,event)teach_Objectcallback(src, self, j));
    end
end

function resize_Objectcallback(self)
    % come here on figure resize events
    fig = gcbo;   % this figure (whose callback is executing)
    fs = get(fig, 'Position');  % get size of figure
    ps = get(self.tc.panel, 'Position');  % get position of the panel
    % update dimensions of the axis area
    set(self.tc.curax, 'Units', 'pixels', ...
        'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
    % keep the panel anchored to the top left corner
    set(self.tc.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
end

function quit_Objectcallback(self)
    set(self.tc.fig, 'ResizeFcn', '');
    delete(self.tc.panel);
    set(self.tc.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
end

function AddExitButtonObject(self)
    %---- add buttons
    uicontrol(self.tc.panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.80 self.tc.height*(0)+.01 0.15 self.tc.height], ...
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
    uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.tc.bgcol, ...
        'Position', [0.05 1-5*self.tc.height 0.2 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', self.tc.sliderLabels(4));
    
    self.tc.t6.r(1) = uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-5*self.tc.height 0.6 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', rad2deg(self.tc.T6(1,3))));    % Degrees
    
    % P
    uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.tc.bgcol, ...
        'Position', [0.05 1-6*self.tc.height 0.2 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', self.tc.sliderLabels(5));
    
    self.tc.t6.r(2) = uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-6*self.tc.height 0.6 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', rad2deg(self.tc.T6(2,3))));    % Degrees
    
    % Y
    uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.tc.bgcol, ...
        'Position', [0.05 1-7*self.tc.height 0.2 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', self.tc.sliderLabels(6));
    
    self.tc.t6.r(3) = uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-7*self.tc.height 0.6 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', rad2deg(self.tc.T6(3,3))));    % Degrees 
end

function CreateObjectPositionDisplay(self)
    %---- set up the position display box 
    % X
    uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.tc.bgcol, ...
        'Position', [0.05 1-self.tc.height 0.2 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'x:');
    
    self.tc.t6.t(1) = uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-self.tc.height 0.6 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', self.tc.T6(1,4)), ...
        'Tag', 'T6');
    
    % Y
    uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.tc.bgcol, ...
        'Position', [0.05 1-2*self.tc.height 0.2 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'y:');
    
    self.tc.t6.t(2) = uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-2*self.tc.height 0.6 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', self.tc.T6(2,4)));
    
    % Z
    uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', self.tc.bgcol, ...
        'Position', [0.05 1-3*self.tc.height 0.2 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'z:');
    
    self.tc.t6.t(3) = uicontrol(self.tc.panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-3*self.tc.height 0.6 self.tc.height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', self.tc.T6(3,4)));
end

function MakeObjectSliders(self)
    %---- sliders
    % Create the sliders
    n = size(self.tc.sliderLabels,2);
    for j=1:n
        % slider label
        uicontrol(self.tc.panel, 'Style', 'text', ...
            'Units', 'normalized', ...
            'BackgroundColor', self.tc.bgcol, ...
            'Position', [0 self.tc.height*(n-j+2) 0.15 self.tc.height], ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.5, ...
            'String', sprintf(self.tc.sliderLabels(:,j)));
        
        % slider itself
        switch(j)
            case {1,2,3}
                % Get XYZ positions
                val = self.tc.T6(j,4);
            case {4,5,6}
                % Get the angles                    % Degrees
                val = rad2deg(self.tc.T6(j-3,3));
        end
        self.tc.slider(j) = uicontrol(self.tc.panel, 'Style', 'slider', ...
            'Units', 'normalized', ...
            'Position', [0.15 self.tc.height*(n-j+2) 0.55 self.tc.height], ...
            'Min', self.tc.sliderLimits(j,1), ...
            'Max', self.tc.sliderLimits(j,2), ...
            'Value', val, ...
            'Tag', sprintf('Slider%d', j));
        
        % text box showing slider value, also editable
        self.tc.edit(j) = uicontrol(self.tc.panel, 'Style', 'edit', ...
            'Units', 'normalized', ...
            'Position', [0.70 self.tc.height*(n-j+2)+.01 0.30 0.9*self.tc.height], ...
            'BackgroundColor', self.tc.bgcol, ...
            'String', num2str(val, 3), ...
            'HorizontalAlignment', 'left', ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.4, ...
            'Tag', sprintf('Edit%d', j));
    end
end

function GetObjectTransform(self)
    %---- get the current object pose
    self.tc.T6 = self.myFkine(self.qCurrent);
end

function InstallTheObjectPanel(self)
    %---- install the panel at the side of the figure
    % find the right figure to put it in
    c = findobj(gca, 'Tag', self.model.name);      % check the current axes
    if isempty(c)
        % doesn't exist in current axes, look wider
        c = findobj(0, 'Tag', self.model.name);    % check all figures
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
    self.tc.fig = get(ax, 'Parent');                % get the figure that holds the axis
    
    % shrink the current axes to make room
    %   [l b w h]
    set(ax, 'OuterPosition', [0.25 0 0.70 1])
    
    self.tc.curax = ax;
    
    % create the panel itself
    self.tc.panel = uipanel(self.tc.fig, ...
        'Title', 'Teach Cartesian', ...
        'BackGroundColor', self.tc.bgcol,...
        'Position', [0 0 .25 1]);
    set(self.tc.panel, 'Units', 'pixels'); % stop automatic resizing
    set(self.tc.fig, 'Units', 'pixels');
    set(self.tc.fig, 'ResizeFcn', @(src,event) resize_Objectcallback(self));
end

