classdef Simulation < handle
    % Vars
    properties
        logObj;         % Log object
        simRunning;     % Flag for simulation desired state
        envObjList;
        dominosTotal;
    end
    % Const Vars
    properties(Constant)
        GENERIC = 1;
        TABLE = 2;
        MYCOBOT = 3;
        STOPBUTTON = 4;
        EXTINGUISHER = 5;
        DOMINO = 6;
    end
    
    methods
        % Constructor
        function self = Simulation(logArg, dominosTotal)
            self.logObj = logArg;                   % Store log object
            self.logObj.LogInfo('[SIM] Simulation constructor');
            self.dominosTotal = dominosTotal;       % Set the number of dominos
            genericList = {};
            tableList = {};
            myCobotList = {};
            stopButtonList = {};
            extinguisherList = {};
            dominoList = {};
            self.envObjList = {genericList, tableList, myCobotList, stopButtonList, extinguisherList, dominoList}
        end
        
        %Deconstructor
        function delete(self)
            self.logObj.LogInfo('[SIM] Simulation deconstructor');
        end
        
        % Function to Generate walls & floor surfaces
        function GenerateEnvironment(self)
            self.logObj.LogDebug('[SIM] GenerateEnvironment()');
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
        
        % Function to add environment objects to the object list
        function AddEnvironmentObject(self, envObj)
           switch envObj.type
               case 'table'
                   id = numel(self.envObjList{self.TABLE} ) + 1;
                   self.envObjList{self.TABLE}{id} = envObj; 
               case 'mycobot'
                   id = numel(self.envObjList{self.MYCOBOT} ) + 1;
                   self.envObjList{self.MYCOBOT}{id} = envObj; 
               case 'stopButton'
                   id = numel(self.envObjList{self.STOPBUTTON} ) + 1;
                   self.envObjList{self.STOPBUTTON}{id} = envObj; 
               case 'extinguisher'
                   id = numel(self.envObjList{self.EXTINGUISHER} ) + 1;
                   self.envObjList{self.EXTINGUISHER}{id} = envObj; 
               case 'domino'
                   id = numel(self.envObjList{self.DOMINO} ) + 1;
                   self.envObjList{self.DOMINO}{id} = envObj; 
               otherwise
                   id = numel(self.envObjList{self.GENERIC} ) + 1;
                   self.envObjList{self.GENERIC}{id} = envObj; 
           end
        end
        
        % Function to Spawn environment objects
        function SpawnEnvironmentObjects(self)
            self.logObj.LogDebug('[SIM] SpawnEnvironmentObjects()');
            
            % Table Object
            tablePose = {transl(0,0,1) * trotz(pi/2)};                                          % Table Pose
            for i = 1:numel(tablePose)
                self.AddEnvironmentObject(Table(self.logObj, i, tablePose{i}));                 % Spawn single object
            end

            % MyCobot Object
            MyCobotPose = {transl(1, 1, 1)};                            % MyCobot Pose
            for i = 1:numel(MyCobotPose)
                self.AddEnvironmentObject(MyCobot(self.logObj, i, MyCobotPose{i}));             % Spawn single object
            end
            
            % StopButton Objects
            StopButtonPose = {transl(1.2,1.8,1),transl(-1.2,1.8,1)};                            % StopButton Poses
            for i = 1:numel(StopButtonPose)
                self.AddEnvironmentObject(StopButton(self.logObj, i, StopButtonPose{i}));       % Spawn single object 
            end
                        
            % Extinguisher Objects
            ExtinguisherPose = {transl(1.2,2.2,0),transl(-1.2,2.2,0)};                          % Extinguisher Poses
            for i = 1:numel(ExtinguisherPose)
                self.AddEnvironmentObject(Extinguisher(self.logObj, i, ExtinguisherPose{i}));   % Spawn single object 
            end
            
            % Domino Objects
            dominoZOffset = 0.025;   %meters offset as centrepoint is standard pose
            robotReach = 0.28;       %280 mm range of motion from MyCobot manual
            for i = 1:self.dominosTotal
                randX = rand();
                randY = rand();
                % This is crap, in theory could loop forever & doesnt check
                % it collides with something existing
                while(randX>robotReach)
                    randX = rand();
                end
                while(randY>robotReach)
                    randY = rand();
                end
                DominoPose = transl(0+rand(), 0+rand(), 1+dominoZOffset);               % Domino Poses
                self.AddEnvironmentObject(Domino(self.logObj, i, DominoPose));                  % Spawn single object 
            end
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
                    self.logObj.LogDebug('[SIM] Sim Running');    
               end
            end
             self.logObj.LogInfo('[SIM] Simulation Stopped');
        end
        
    end
end