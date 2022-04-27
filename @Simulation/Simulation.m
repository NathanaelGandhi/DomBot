classdef Simulation < handle
    % Vars
    properties
        simulation;     % Handle
        cliDebug;       % Flag for CLI Debug
    end
    % Const Vars
    properties(Constant)

    end
    
    methods
        % Constructor
        function self = Simulation(cliDebugVal)
            self.cliDebug = cliDebugVal;
            if(self.cliDebug); disp("[SIM] Simulation constructor"); end
        end
        %Deconstructor
        function delete(self)
            if(self.cliDebug); disp("[SIM] Simulation deconstructor"); end
        end
    end
end