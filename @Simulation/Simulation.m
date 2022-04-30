classdef Simulation < handle
    % Vars
    properties
        simulation;     % Handle
        logOb;          % Log object
        simRunning      % Flag for simulation desired state

    end
    % Const Vars
    properties(Constant)

    end
    
    methods
        % Constructor
        function self = Simulation(logArg)
            self.logOb = logArg;    % Store log object
            self.logOb.LogInfo('[SIM] Simulation constructor');
            %GenerateEnvironment();
        end
        
        %Deconstructor
        function delete(self)
            self.logOb.LogInfo('[SIM] Simulation deconstructor');
        end
        
        % Function to Generate walls & floor surfaces
        function GenerateEnvironment(self)
            self.logOb.LogDebug('[SIM] GenerateEnvironment()');
            % Generate Environment
            hold off
            % Concrete Floor
            surf([-4,-4;4,4],[-4,4;-4,4],[0,0;0,0],'CData',imread('assets/concrete.jpg'),'FaceColor','texturemap');
            hold on
            % Side Walls - remove top for viewing
            surf([4,4;4,4],[-4,4;-4,4],[3,3;0,0],'CData',imread('assets/SideWall.jpg'),'FaceColor','texturemap');
            surf([-4,-4;-4,-4],[-4,4;-4,4],[3,3;0,0],'CData',imread('assets/SideWall.jpg'),'FaceColor','texturemap');
            % Back Wall
            surf([-4,4;-4,4],[-4,-4;-4,-4],[3,3;0,0],'CData',imread('assets/BackWall.jpg'),'FaceColor','texturemap');
        end
        
        % Function to set the simulation running flag
        function SetSimRunningFlag(self, flag)
            self.simRunning = flag;
        end
        
        % Function to run simulation "main" loop
        function RunSim(self)
            while (self.simRunning)
               % Sim running. Loop while flag condition is true 
               if (self.simRunning)
                   pause(1);
                    self.logOb.LogDebug('[SIM] Sim Running');    
               end
            end
             self.logOb.LogInfo('[SIM] Simulation Stopped');
        end
        
    end
end