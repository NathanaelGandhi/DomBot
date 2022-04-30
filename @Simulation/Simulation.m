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
%             self@Log();
            LogInfo(self,'[SIM] Simulation constructor');
            %logOb.LogInfo(self,'[SIM]');
            %GenerateEnvironment();
        end
        %Deconstructor
        function delete(self)
            LogInfo(self,'[SIM] Simulation deconstructor');
        end
        function GenerateEnvironment(self)
            LogInfo(self,'[SIM] GenerateEnvironment()');
            disp('[SIM] Disp GenerateEnvironment()');
            %Generate Environment
            %   Detailed explanation goes here
            % Concrete Floor
            hold off
            surf([-4,-4;4,4],[-4,4;-4,4],[0,0;0,0],'CData',imread('../assets/concrete.jpg'),'FaceColor','texturemap');
            hold on
            % Side Walls - remove top for viewing
            surf([4,4;4,4],[-4,4;-4,4],[3,3;0,0],'CData',imread('../assets/SideWall.jpg'),'FaceColor','texturemap');
            surf([-4,-4;-4,-4],[-4,4;-4,4],[3,3;0,0],'CData',imread('../assets/SideWall.jpg'),'FaceColor','texturemap');
            % Back Wall
            surf([-4,4;-4,4],[-4,-4;-4,-4],[3,3;0,0],'CData',imread('../assets/BackWall.jpg'),'FaceColor','texturemap');
        end
    end
end