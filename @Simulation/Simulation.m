classdef Simulation < handle
    % Vars
    properties
        simulation;     % Handle
        logOb;
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
    end
end