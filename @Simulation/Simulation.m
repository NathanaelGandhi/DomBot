classdef Simulation < Log
    % Vars
    properties
        simulation;     % Handle
    end
    % Const Vars
    properties(Constant)

    end
    
    methods
        % Constructor
        function self = Simulation()
            self@Log();
            LogInfo(self,'[SIM] Simulation constructor');
        end
        %Deconstructor
        function delete(self)
            LogInfo(self,'[SIM] Simulation deconstructor');
        end
    end
end