classdef Simulation < handle
    % Vars
    properties
        logObj;         % Log object
        simRunning;     % Flag for simulation desired state
        envObjList;
        dominosTotal;
        simEStop;       % Flag for simulation E-Stop
        
        % Path properties
        pathType;               % Domino path type (Circle, line or semicircle)
        pathStartPt;            % 4x4 transform start point for domino path (world frame)
        pathEndPt;              % 4x4 transform end point for domino path (world frame)
        
    end
    % Const Vars
    properties(Constant)
        GENERIC = 1;
        TABLE = 2;
        MYCOBOT = 3;
        STOPBUTTON = 4;
        EXTINGUISHER = 5;
        DOMINO = 6;
        STOPSIGN = 7;
        ROBOTREACH = 0.28;       %280 mm range of motion from MyCobot manual
        ROBOTBASERADIUS = 0.05;  % Exclusion radius for robot base
        
        % Domino path constants
        CIRCLE = 1;
        SEMICIRCLE = 2;
        LINE = 3;
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
            stopSignList = {};
            self.envObjList = {genericList, tableList, myCobotList, stopButtonList, extinguisherList, dominoList, stopSignList}
        end
        
        %Deconstructor
        function delete(self)
            self.logObj.LogInfo('[SIM] Simulation deconstructor');
        end
        
        %% Function to Generate walls & floor surfaces
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
        
        %% Function to add environment objects to the object list
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
               case 'stopSign'
                   id = numel(self.envObjList{self.STOPSIGN} ) + 1;
                   self.envObjList{self.STOPSIGN}{id} = envObj; 
               otherwise
                   id = numel(self.envObjList{self.GENERIC} ) + 1;
                   self.envObjList{self.GENERIC}{id} = envObj; 
           end
        end
        
        %% Function to Spawn environment objects
        function SpawnEnvironmentObjects(self)
            self.logObj.LogDebug('[SIM] SpawnEnvironmentObjects()');
            
            % Table Object
            tablePose = {transl(0,0,1) * trotz(pi/2)};                                          % Table Pose
            for i = 1:numel(tablePose)
                self.AddEnvironmentObject(Table(self.logObj, i, tablePose{i}));                 % Spawn single object
            end

            % MyCobot Object
            myCobotPose = {transl(0, 0, 1)};                                                  % MyCobot Pose
            for i = 1:numel(myCobotPose)
                self.AddEnvironmentObject(MyCobot(self.logObj, i, myCobotPose{i}));             % Spawn single object
            end
            
            % StopButton Objects
            stopButtonPose = {transl(1.2,1.8,1),transl(-1.2,1.8,1)};                            % StopButton Poses
            for i = 1:numel(stopButtonPose)
                self.AddEnvironmentObject(StopButton(self.logObj, i, stopButtonPose{i}));       % Spawn single object 
            end
                        
            % Extinguisher Objects
            extinguisherPose = {transl(1.2,2.2,0),transl(-1.2,2.2,0)};                          % Extinguisher Poses
            for i = 1:numel(extinguisherPose)
                self.AddEnvironmentObject(Extinguisher(self.logObj, i, extinguisherPose{i}));   % Spawn single object 
            end
            
            % Stop Sign Objects
            stopSignPose = {transl(0.5,0.5,1) * trotx(pi/2)};                                                     % Stop Sign Poses
            for i = 1:numel(stopSignPose)
                self.AddEnvironmentObject(StopSign(self.logObj, i, stopSignPose{i}));           % Spawn single object 
            end
            
            % Domino Objects
            for i = 1:self.dominosTotal
                %[xPose,yPose] = self.GenerateSquareDominoPose();
                [xPose,yPose] = self.GenerateRadiusDominoPose();
                % Generated domino pose is valid
                DominoPose = transl(...
                    self.envObjList{self.MYCOBOT}{1}.pose(13)+xPose, ...
                    self.envObjList{self.MYCOBOT}{1}.pose(14)+yPose, ...
                    self.envObjList{self.MYCOBOT}{1}.pose(15));               % Domino Poses
                self.AddEnvironmentObject(Domino(self.logObj, i, DominoPose));                % Spawn single object
            end
        end
        
        %% Function to generate domino pose using radius exclusion zones
        function [xPose, yPose] = GenerateSquareDominoPose(self)
            randX = -1 + (1--1).*rand();                                % Generate random number between -1 and 1
            randY = -1 + (1--1).*rand();                                % Used as scaling based on next check
            % This is crap, in theory could loop forever & doesnt check
            % it collides with something existing
            % Check if the domino is within reach of the robot
            % Check if the domino is not within the robot base
            % Using a square exclusion zone
            while(randX>self.ROBOTREACH || randX<-self.ROBOTREACH || randX<self.ROBOTBASERADIUS && randX>-self.ROBOTBASERADIUS)
                randX = -1 + (1--1).*rand();
            end
            while(randY>self.ROBOTREACH || randY<-self.ROBOTREACH || randY<self.ROBOTBASERADIUS && randY>-self.ROBOTBASERADIUS)
                randY = -1 + (1--1).*rand();
            end
            xPose = randX;
            yPose = randY;
        end
        
        %% Function to generate domino pose using radius exclusion zones
        function [xPose, yPose] = GenerateRadiusDominoPose(self)
            randX = -1 + (1--1).*rand();                                % Generate random number between -1 and 1
            randY = -1 + (1--1).*rand();                                % Used as scaling based on next check
            testPose = sqrt((randX^2) + (randY^2));
            % This is crap, in theory could loop forever & doesnt check
            % it collides with something existing
            % Check if the domino is within reach of the robot
            % Check if the domino is not within the robot base
            % Using radius exclusion zone
            while(testPose>self.ROBOTREACH || testPose<-self.ROBOTREACH || testPose<self.ROBOTBASERADIUS && testPose>-self.ROBOTBASERADIUS)
                randX = -1 + (1--1).*rand();
                randY = -1 + (1--1).*rand();
                testPose = sqrt(randX^2 + randY^2);
            end
            xPose = randX;
            yPose = randY;
        end
        
        %% Function to set the simulation running flag
        function SetSimRunningFlag(self, flag)
            self.simRunning = flag;
        end
        
        %% Function to set the simulation E-Stop flag
        function SetSimEStopFlag(self, flag)
            self.simEStop = flag;
        end
        
        %% Function to run simulation "main" loop
        function RunSim(self)
            % UPDATE REQIUIRED - changes made for video
            
            % TEST - set start point for path (in robot base frame)
            startPoint = transl(0.2,0,0);
            endPoint = startPoint;
            
            % Set path for dominoes
            SetDominoPath(self, self.CIRCLE, startPoint, endPoint);
            
            % Set goal poses for each domino
            GenerateDominoGoalPoses(self);
            
            % Test to verify correct goal pose calculation (plots dominoes
            % in goal poses)
%             for i = 1:self.dominosTotal
%                 self.envObjList{self.DOMINO}{i}.UpdatePose(self.envObjList{self.DOMINO}{i}.desiredPose);
%             end
            
            while (1)
               % Sim running. Loop while flag condition is true 
               if (self.simEStop)
                   pause(1);
                   self.logObj.LogDebug('[SIM] Sim E-stopped');
                   continue
               elseif (self.simRunning)
                   pause(1);
                   self.logObj.LogDebug('[SIM] Sim Running'); 
               elseif (self.simRunning == 0)
                   break;
               end
            end
             self.logObj.LogInfo('[SIM] Simulation Stopped');
        end
        
        %% Function to start "teach"
        function StartTeach(self)
            self.envObjList{3}{1}.StartTeach();
        end
        
        %% Function to set desired domino path
        function SetDominoPath(self, pathInput, startPt, endPt)
            % UPDATE REQUIRED
            % - If, elseif statement used as further modification will
            % require this later on. 
            % - Program does not check for path validity of the line,
            % meaning that it can pass through the robot base.
            % - Function will only work with one robot, and requires
            % modification if intending to implement more robots.
            
            % NOTE - Path inputs are specified from the robots base frame
            
            % Log the sim state
            self.logObj.LogInfo('[SIM] Setting Domino Path');
            
            % Set the domino path based on user input
            if pathInput == self.CIRCLE
                self.pathType = pathInput;
                self.pathStartPt = self.envObjList{self.MYCOBOT}{1}.model.base * startPt;
                self.pathEndPt = self.envObjList{self.MYCOBOT}{1}.model.base * endPt; %% EndPt will equal startPT for circle
            elseif pathInput == self.SEMICIRCLE
                self.pathType = pathInput;
                self.pathStartPt = self.envObjList{self.MYCOBOT}{1}.model.base * startPt;
                self.pathEndPt = self.envObjList{self.MYCOBOT}{1}.model.base * endPt;
            elseif pathInput == self.LINE
                self.pathType = pathInput;
                self.pathStartPt = self.envObjList{self.MYCOBOT}{1}.model.base * startPt;
                self.pathEndPt = self.envObjList{self.MYCOBOT}{1}.model.base * endPt;
            else %ERROR
                self.logObj.LogInfo('[SIM] ERROR - Domino path not specified');
            end
        end
        
        %% Function to generate goal domino poses
        function GenerateDominoGoalPoses(self)
            % UPDATE REQUIRED - functional for trailer video
            % - program does not check for the quantity of dominoes, and
            % whether this is valid for the selected path (i.e. the user
            % can select to have a circle path with 4 dominoes. Suggest
            % correction of GUI to have a minimum number of 4 dominoes.
            
            % NOTE - Dominoes are placed along the path at intervals determined by
            % the amount of dominoes selected by the user. The start point
            % will be occupied, but the end point will not to avoid
            % overlap.
            
            % Log the sim state
            self.logObj.LogInfo('[SIM] Calculating Domino Poses');
            
            % Determine domino goal poses using stored data
            if self.pathType == self.CIRCLE
                % Set a path around the robot with the specified startPt
                
                % Determine radius as distance from startPt to robot base
                pathRadius = hypot(self.pathStartPt(13),self.pathStartPt(14));
                
                % Determine distance around circle for each domino
                goalTransIncrement = (2*pi*pathRadius)/self.dominosTotal;
                
                % Determine the increment 
                goalAngleIncrement = 360/self.dominosTotal;
                
                % Set the transform for the first domino (in world frame)
                pathTF = self.pathStartPt;
                
                % Add goal poses to dominoes (in world frame)
                for i = 1:self.dominosTotal
                    
                    % Set the goal pose of the current domino
                    self.envObjList{self.DOMINO}{i}.desiredPose = pathTF;
                    
                    % Determine the goal pose of the next domino
                    pathTF = pathTF * trotz(deg2rad(goalAngleIncrement/2)) * transl(0, goalTransIncrement, 0) * ...
                        trotz(deg2rad(goalAngleIncrement/2));
                    
                end
                
            elseif self.pathType == self.SEMICIRCLE
                % Empty for now
                
            elseif self.pathType == self.LINE
                % Empty for now
                
            else % ERROR
                self.logObj.LogInfo('[SIM] ERROR - Domino path not set');
            end
            
        end
    end
end