classdef Simulation < handle
    % Vars
    properties
        % Handle
        simulation;
    end
    % Const Vars
    properties(Constant)
        CLI_DEBUG = true;
    end
    
    methods
        function self = Simulation()
            if(CLI_DEBUG); disp("Simulation constructor");end
        end
    end
end