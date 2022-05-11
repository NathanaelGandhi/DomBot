function teachCartesian(robot)
%TEACHCARTESIAN Jog the robot using Cartesian coordinates
    %-------------------------------
    % parameters for teach panel
    bgcol = [103 233 98]/255;  % background color
    height = 0.06;  % height of slider rows
    %-------------------------------
    handles = InstallThePanel(robot,bgcol);
    

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
        'Title', 'Teach', ...
        'BackGroundColor', bgcol,...
        'Position', [0 0 .25 1]);
    set(panel, 'Units', 'pixels'); % stop automatic resizing
    handles.panel = panel;
    set(handles.fig, 'Units', 'pixels');
    set(handles.fig, 'ResizeFcn', @(src,event) resize_callback(robot.model, handles));
end

