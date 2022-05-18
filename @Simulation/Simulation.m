classdef Simulation < handle
    % Vars
    properties
        logObj;         % Log object
        objList;        % List of objects
        simRunning;     % Flag for simulation desired state
        simEStop;       % Flag for simulation E-Stop
        dominosTotal;   % Total No. of dominoes
        stepsCurrent;   % Current steps for robot
        
        % Path properties
        pathType;       % Domino path type (Circle, line or semicircle)
        pathStartPt;    % 4x4 transform start point for domino path (world frame)
        pathEndPt;      % 4x4 transform end point for domino path (world frame)
        
        % Domino Collection properties
        dominoFlag;     % Flag for if robot is holding domino
        robotState;     % State of the arm (picking up domino, going home, etc)
        prevState;      % Previous state of the arm
        oldState;       % 2nd previous state of arm
        dominoCurrent;  % Current domino being targeted by the robot
        targetPose;     % Target pose of the robot
        collisionQ;     % Joint angles of the robot at the collision
        goalQ;          % Joint angles of goal for collision avoidance
    end
    
    % Const Vars
    properties(Constant)
        % Object enum
        GENERIC = 1;
        TABLE = 2;
        MYCOBOT = 3;
        STOPBUTTON = 4;
        EXTINGUISHER = 5;
        DOMINO = 6;
        STOPSIGN = 7;
        BARRIER = 8;
        LIGHT = 9;
        PERSON = 10;
        
        ROBOTREACH = 0.28;       %280 mm range of motion from MyCobot manual
        ROBOTBASERADIUS = 0.15;  % Exclusion radius for robot base
        DOMINOMAX = 15;             % Max no. of dominoes for path generation
        DOMINOMIN = 45;             % Min no. of dominoes for path generation
        
        % Domino path States
        CIRCLE = 1;
        SEMICIRCLE = 2;
        LINE = 3;
        
        % Robot Movement States
        THINKING = 0;
        HOVERPOSE = 1;
        DOMINOPOSE = 2;
        HOMEPOSE = 3;
        RUNNING = 4;
        STANDBY = 5;
        
        % myCobot Constants
        ROBOTHOME = [0, -pi/8, -pi/2, pi/8, -pi/2, 0];  % Home Pose
        ROBOTSTANDBY = [0, 0, 0, -pi/2, -pi/2, 0];      % Standby pose
        ROBOTHOVER = [0, -pi/4, -pi/3, pi/8, -pi/2, 0]; % TEMP
        ROBOTHOVEROFFSET = 0.07;                        % Offset from top of domino for hover pose
        ROBOTEEOFFSET = 0.05;                           % Offset of EE to Domino
        STEPSTOTAL = 50;                                % Total steps for robot movement
        
        % Domino States
        FREE = 0;
        OCCUPIED = 1;
        PLACED = 2;
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
            barrierList = {};
            lightList = {};
            personList = {};
            self.objList = {genericList, tableList, myCobotList, ...
                stopButtonList, extinguisherList, dominoList, ...
                stopSignList, barrierList, lightList, personList};
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
            
            % Generate Safety 
            % Floor Exclusion Zone
            surf([-3,-3;3,3],[-2.1,2.1;-2.1,2.1],[0.01,0.01;0.01,0.01],'CData',imread('assets/ExclusionZone.png'),'FaceColor','texturemap');
            % Safety Signs
            surf([2.1,1.4;2.1,1.4],[3.95,3.95;3.95,3.95],[0.8,0.8;0.35,0.35],'CData',imread('assets/WarningSign.jpg'),'FaceColor','texturemap');
            surf([-1.4,-2.1;-1.4,-2.1],[3.95,3.95;3.95,3.95],[0.8,0.8;0.35,0.35],'CData',imread('assets/WarningSign.jpg'),'FaceColor','texturemap');
        end
        
        %% Function to add environment objects to the object list
        function AddEnvironmentObject(self, envObj)
           switch envObj.type
               case 'table'
                   id = numel(self.objList{self.TABLE} ) + 1;
                   self.objList{self.TABLE}{id} = envObj; 
               case 'mycobot'
                   id = numel(self.objList{self.MYCOBOT} ) + 1;
                   self.objList{self.MYCOBOT}{id} = envObj; 
               case 'stopButton'
                   id = numel(self.objList{self.STOPBUTTON} ) + 1;
                   self.objList{self.STOPBUTTON}{id} = envObj; 
               case 'extinguisher'
                   id = numel(self.objList{self.EXTINGUISHER} ) + 1;
                   self.objList{self.EXTINGUISHER}{id} = envObj; 
               case 'domino'
                   id = numel(self.objList{self.DOMINO} ) + 1;
                   self.objList{self.DOMINO}{id} = envObj; 
               case 'stopSign'
                   id = numel(self.objList{self.STOPSIGN} ) + 1;
                   self.objList{self.STOPSIGN}{id} = envObj; 
               case 'barrier'
                   id = numel(self.objList{self.BARRIER} ) + 1;
                   self.objList{self.BARRIER}{id} = envObj; 
               case 'light'
                   id = numel(self.objList{self.LIGHT} ) + 1;
                   self.objList{self.LIGHT}{id} = envObj; 
               case 'person'
                   id = numel(self.objList{self.PERSON} ) + 1;
                   self.objList{self.PERSON}{id} = envObj; 
               otherwise
                   id = numel(self.objList{self.GENERIC} ) + 1;
                   self.objList{self.GENERIC}{id} = envObj; 
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
            myCobotPose = {transl(0, 0, 1)};                                                    % MyCobot Pose
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
            stopSignPose = {transl(0.5,0.5,1) * trotx(pi/2)};                                   % Stop Sign Poses
            for i = 1:numel(stopSignPose)
                self.AddEnvironmentObject(StopSign(self.logObj, i, stopSignPose{i}));           % Spawn single object 
            end
            
            % Barrier Objects
            barrierPose = {transl(-3.25,3.875,0),transl(-1.75,3.875,0),...
                transl(1.75,3.875,0),transl(3.25,3.875,0)};                                     % Barrier Poses
            for i = 1:numel(barrierPose)
                self.AddEnvironmentObject(Barrier(self.logObj, i, barrierPose{i}));             % Spawn single object 
            end
            
            % Light Objects
            lightPose = {transl(0,1.8,1)};                                                      % Light Poses
            for i = 1:numel(lightPose)
                self.AddEnvironmentObject(Light(self.logObj, i, lightPose{i}));                 % Spawn single object 
            end
            
            % Person Objects
            personPose = {transl(0,5,1) * trotx(pi/2)};                                                       % Person Poses
            for i = 1:numel(personPose)
                self.AddEnvironmentObject(Person(self.logObj, i, personPose{i}));               % Spawn single object 
            end
            
            % Domino Objects
            for i = 1:self.dominosTotal
                %[xPose,yPose] = self.GenerateSquareDominoPose();
                [xPose,yPose] = self.GenerateRadiusDominoPose();
                % Generated domino pose is valid
                DominoPose = transl(...
                    self.objList{self.MYCOBOT}{1}.pose(13)+xPose, ...
                    self.objList{self.MYCOBOT}{1}.pose(14)+yPose, ...
                    self.objList{self.MYCOBOT}{1}.pose(15));                        % Domino Poses
                self.AddEnvironmentObject(Domino(self.logObj, i, DominoPose));      % Spawn single object
                self.objList{self.DOMINO}{i}.dominoState = self.FREE;               % Set as available for pickup logic
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
        function SetUpSim(self,selectedPath)     
            % Set path for dominoes - change CIRCLE to SEMICIRCLE or LINE
            % for other paths - LINE does not work right now
            SetDominoPath(self, selectedPath);
            
            % Set goal poses for each domino
            GenerateDominoGoalPoses(self);
            
            % LEAVE IN FOR TRAVIS DEBUG
            % TEST - Verify correct goal pose calculation (plots dominoes
            % in goal poses)
%             for i = 1:self.dominosTotal
%                 self.objList{self.DOMINO}{i}.UpdatePose(self.objList{self.DOMINO}{i}.desiredPose);
%             end
            
            % Set up robot movement parameters
            self.dominoCurrent = 1;                 % Set the current domino at 1
            self.robotState = self.HOMEPOSE;        % Set the current state to go to home
            self.stepsCurrent = 0;                  % Current step is 0, iterated when running
            self.dominoFlag = self.FREE;            % Robot is not holding a domino
        end
        
        %% Function to run robot - state machine
        % @robot Robot Number
        function RunRobot(self,robot)
            % Runs the robot using the steps, domino count and states to
            % pick up and drop off dominoes.
            % CAUTION - this is a big function, and its pretty spaghetti.
            % Oops...
            
            % State machine - the brains behind the movement
            switch self.robotState
                
                % Determining domino for the arm to target
                case self.THINKING
                    % Checking domino array for available dominoes
                    for i = 1:self.dominosTotal
                        % If available domino found
                        if self.objList{self.DOMINO}{i}.dominoState == self.FREE
                            % Protect
                            self.objList{self.DOMINO}{i}.dominoState = self.OCCUPIED;
                            % Set current domino as target
                            self.dominoCurrent = i;
                            % Set the program to determine the path and record last state
                            self.oldState = self.prevState;
                            self.prevState = self.robotState;
                            self.robotState = self.HOVERPOSE;
                            self.logObj.LogInfo('[SIM] Thinking - Moving to free domino');
                            self.logObj.LogInfo(sprintf('[SIM] Target domino: %s',num2str(self.dominoCurrent)));
                            % Break the loop
                            break;
                        elseif self.objList{self.DOMINO}{self.dominosTotal}.dominoState == self.OCCUPIED
                            % If the robot finds no available dominos, go
                            % to standby - job done!
                            self.robotState = self.HOMEPOSE;
                            self.logObj.LogInfo('[SIM] Thinking - No free dominos');
                            break;
                        end
                    end
                    
                % Plan path to the home or standby pose of the robot
                case self.HOMEPOSE
                    if self.objList{self.DOMINO}{1}.dominoState == self.FREE
                        % Determine the trajectory to the home pose
                        self.objList{self.MYCOBOT}{robot}.JTraJ(self.ROBOTHOME, self.STEPSTOTAL);
                        % Set the program to run and record last state
                        % self.oldState = self.prevState; - not needed here
                        self.prevState = self.robotState;
                        self.robotState = self.RUNNING;
                        self.logObj.LogInfo('[SIM] Going to Home');
                    else
                        % Determine the trajectory to the standby pose
                        self.objList{self.MYCOBOT}{robot}.JTraJ(self.ROBOTSTANDBY, self.STEPSTOTAL);
                        % Set the program to run and record last state
                        self.prevState = self.STANDBY;
                        self.robotState = self.RUNNING;
                        self.logObj.LogInfo('[SIM] Going to standby');
                        % Needs a way to be 'turned off'
                    end
                
                % Plan path for hovering over domino current or goal pose
                case self.HOVERPOSE
                    % Set the hover pose - either goal or current
                    if (self.prevState == self.DOMINOPOSE || self.prevState == self.THINKING)
                        pose = self.objList{self.DOMINO}...
                        {self.dominoCurrent}.pose;
                        self.logObj.LogInfo('[SIM] Hovering - above current');
                    else
                        pose = self.objList{self.DOMINO}...
                        {self.dominoCurrent}.desiredPose;
                        self.logObj.LogInfo('[SIM] Hovering - above goal');
                    end
                    % Determine estimate for ikcon
                    estimatePose = self.ROBOTHOVER;
                    estimatePose(1) = atan2(pose(14), pose(13));
                    % Determine the joint angles for the current pose
                    qGoal = self.objList{self.MYCOBOT}{robot}.model.ikcon(pose * transl(0,0,self.ROBOTEEOFFSET + self.ROBOTHOVEROFFSET), ...
                        estimatePose);
                    % Determine the trajectory to the home pose
                    self.objList{self.MYCOBOT}{robot}.JTraJ(qGoal, self.STEPSTOTAL);
                    % Set the program to run and record last stated
                    self.oldState = self.prevState;
                    self.prevState = self.robotState;
                    self.robotState = self.RUNNING;
                
                % Plan path for moving to domino current or goal pose
                case self.DOMINOPOSE
                    % Set the domino pose - either goal or current
                    if self.dominoFlag == self.OCCUPIED
                        pose = self.objList{self.DOMINO}...
                        {self.dominoCurrent}.desiredPose;
                        self.logObj.LogInfo('[SIM] Domino - setting down');
                    else
                        pose = self.objList{self.DOMINO}...
                        {self.dominoCurrent}.pose;
                        self.logObj.LogInfo('[SIM] Domino - Picking up');
                    end
                    % Determine the joint angles for the current pose
                    qGoal = self.objList{self.MYCOBOT}{robot}.model.ikcon(pose * transl(0,0,self.ROBOTEEOFFSET), ...
                        self.objList{self.MYCOBOT}{robot}.model.getpos);
                    % Determine the trajectory to the home pose
                    self.objList{self.MYCOBOT}{robot}.JTraJ(qGoal, self.STEPSTOTAL);
%                     self.objList{self.MYCOBOT}{robot}.CalculateTraj(pose * transl(0,0,self.ROBOTEEOFFSET), self.STEPSTOTAL);
                    % Set the program to run and record last stated
                    self.oldState = self.prevState;
                    self.prevState = self.robotState;
                    self.robotState = self.RUNNING;
                    
                    
                case self.RUNNING
                    % Runs trajectory that was just calculated
                    self.objList{self.MYCOBOT}{robot}.RunTraj();
                    
                    % Increment the step count
                    self.stepsCurrent = self.stepsCurrent + 1;
                    
                    % If the domino is held, move the domino
                    if self.dominoFlag == self.OCCUPIED
                        % determine transform for domino
                        dominoTF = self.objList{self.MYCOBOT}{robot}.model.fkine(self.objList{self.MYCOBOT}{robot}.model.getpos) * ...
                            transl(0, 0, -1 * self.ROBOTEEOFFSET);
                        self.objList{self.DOMINO}{self.dominoCurrent}.UpdatePose(dominoTF);
                    end
                    
                    % If max steps reached, the move is completed - go to next
                    % step in the state machine
                    if self.stepsCurrent == self.STEPSTOTAL
                        
                        % Reset step count
                        self.stepsCurrent = 0;
                        
                        % Logic for next state
                        % State 1 - No dominoes moved and robot thinking
                        if (self.prevState == self.HOMEPOSE)
                            % Find first free domino
                            self.robotState = self.THINKING;
                            
                        % State 2 - Hovering over domino current or goal
                        elseif (self.prevState == self.HOVERPOSE && self.oldState == self.THINKING)
                            self.robotState = self.DOMINOPOSE;
                            
                        % State 3 - 
                        elseif (self.prevState == self.DOMINOPOSE && self.oldState == self.HOVERPOSE)
                            self.robotState = self.HOVERPOSE;
                            self.dominoFlag = self.OCCUPIED - self.dominoFlag;
                            
                        % State 4 - Hovering over domino current or goal
                        elseif (self.prevState == self.HOVERPOSE && self.oldState == self.DOMINOPOSE)
                            if self.dominoFlag == self.FREE
                                self.robotState = self.THINKING;
                            else
                                self.robotState = self.HOVERPOSE;
                            end
                            
                        % State 5 - 
                        elseif (self.prevState == self.HOVERPOSE && self.oldState == self.HOVERPOSE)
                            self.robotState = self.DOMINOPOSE;
                            
                        elseif (self.prevState == self.STANDBY)
                            self.robotState = self.STANDBY;
                            pause(0.1);
                        else
                            self.logObj.LogInfo('[SIM] ERROR');
                            
                        end
                    end
                case self.STANDBY
                    self.dominoCurrent = 1;                 % Set the current domino at 1
                    self.stepsCurrent = 0;                  % Current step is 0, iterated when running
                    self.dominoFlag = self.FREE;            % Robot is not holding a domino
                    
            end
            
        end
        
        %% Function to run simulation "main" loop
        function RunSim(self)
            % Main loop for code - runs the robot unless e-stopped
            collisionCheckFlag = 0;
            
            while (~self.simEStop)
                while (self.simRunning)
                    if (self.objList{self.PERSON}{1}.pose(14) < 4)
                        pause(0.5);
                    else
                        RunRobot(self,1);    % Run robot 1
                        collisionCheckFlag = collisionCheckFlag + 1;
                        % Check for collisions with the stop sign
                        if collisionCheckFlag > 10
                            CollisionAvoidance(self);
                            collisionCheckFlag = 0;
                        
                        end
                    end
                    % Run robot state machine
                    
%                     while (self.objList{self.PERSON}{1}.pose(14) < 4)
%                         pause(1);
%                     end
                end
                % Log E-Stop activation
                self.logObj.LogInfo('[SIM] Simulation Stopped');
                pause(1);
            end
        end
        
        %% RobotRetreat
        % Just place this function in a while loop
        function RobotRetreat(self)
            if self.objList{self.MYCOBOT}{1}.searchOrRetreatFlag
                self.objList{self.MYCOBOT}{1}.robotRetreat(self.objList{self.STOPSIGN}{1});
            else
                self.objList{self.MYCOBOT}{1}.searchForStopSign(self.objList{self.STOPSIGN}{1});
            end
        end
        
        %% Function to start "teach classic"
        function StartClassicTeach(self)
            self.objList{self.MYCOBOT}{1}.StartClassicTeach();
        end
        
        %% Function to start "teach cartesian"
        function StartCartesianTeach(self)
            self.objList{self.MYCOBOT}{1}.StartCartesianTeach();
        end
        
        %% Function to start "teach object"
        function StartObjectTeach(self)
            self.objList{self.STOPSIGN}{1}.StartObjectTeach();
        end
        
        %% Function to start "teach person"
        function StartPersonTeach(self)
            self.objList{self.PERSON}{1}.StartPersonTeach();
        end
        
        %% Function for collision avoidance
        function CollisionAvoidance(self)            
            % If the sim is running, a qMatrix has been computed and can be
            % used to check for collision
            if self.robotState == self.RUNNING
                % Check for collisions between myCobot and the stop sign
                if (myIsCollision(self))  % IsCollision() replacement   
                    % Log the collision
                    self.logObj.LogInfo('[SIM] Collision Detected, Rerouting');
                    % Reset step count
                    self.stepsCurrent = 0;
                    % Recalculate traj and pass into myCobot class - sort
                    % of works
                    trajA = jtraj(...
                        self.objList{self.MYCOBOT}{1}.model.getpos,...
                        [self.collisionQ(1),0, 0, -pi/2, -pi/2, 0], ...
                        self.STEPSTOTAL/2);
                    trajB = jtraj(...
                        [self.collisionQ(1),0, 0, -pi/2, -pi/2, 0],...
                        self.goalQ, ...
                        self.STEPSTOTAL/2);
                    self.objList{self.MYCOBOT}{1}.qMatrix = [trajA;trajB];
                    %self.objList{self.MYCOBOT}{1}.JTraJ([self.collisionQ(1),0, 0, -pi/2, -pi/2, 0], self.STEPSTOTAL);
                    
                end
            end
        end
        
        %% Function to check if IsCollision
        function result = myIsCollision(self)
            % Get parameters for collision checking
            robot = self.objList{self.MYCOBOT}{1};                          % Current robot
            qMatrix = self.objList{self.MYCOBOT}{1}.qMatrix;                % Current trajectory
            faces = self.objList{self.STOPSIGN}{1}.model.Faces;             % Faces, linked to vertices, of stop sign (obstacle)
            vertex = self.objList{self.STOPSIGN}{1}.model.Vertices;         % Vertices of stop sign (obstacle)
            faceNormals = self.objList{self.STOPSIGN}{1}.model.FaceNormals; % Normal vectors for each face of the stop sign
            
            % Set up function to return once a collision is detected to speed
            % up processing time.
            returnOnceFound = true;
            % Set result flag to be false
            result = false;
            
            % Iterate through qMatrix to check every position
            for qIndex = 2:size(qMatrix,1)
                % Get the TF for every link (stored in linkTF) - eeTF
                % disregarded here as it is also stored in linkTF(4x4x6)
                [eeTF, linkTF] = robot.model.fkine(qMatrix(qIndex,:));
                self.collisionQ = qMatrix(qIndex,:);

                % Go through each link of myCobot
                for i = 2 : size(linkTF,3)-1  
                    % Go through each face (calculated as a plane) for each link
                    for faceIndex = 1:size(faces,1)
                        % Find a vertex on the plane (one vertex of the
                        % current triangle
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        % Check for intersection with the plane
%                         [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,linkTF(1:3,4,i)',linkTF(1:3,4,i+1)'); 
                        planeNormal = faceNormals(faceIndex,:);
                        pointOnPlane = vertOnPlane;
                        point1OnLine = linkTF(1:3,4,i)';
                        point2OnLine = linkTF(1:3,4,i+1)';
                        [intersectionPoint,check] = myLinePlaneIntersection(self,planeNormal,pointOnPlane,point1OnLine,point2OnLine);
                        % Check the returned array for intersections and
                        % flag true.
                        intersectP = intersectionPoint;
                        triangleVerts = vertex(faces(faceIndex,:)',:);
                        if check == 1 && myIsIntersectionPointInsideTriangle(self,intersectP,triangleVerts)
%                         if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%                             plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%                             display('Intersection');
                            % Intersection found.
                            % Set end goalQ of collision traj.
                            self.goalQ = qMatrix(size(qMatrix,1),:);
                            result = true;
                            if returnOnceFound
                                return
                            end
                        end
                    end    
                end
            end
        end
        
        %% Function to check IsIntersectionPointInsideTriange
        function result = myIsIntersectionPointInsideTriangle(self,intersectP,triangleVerts)
            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);

            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);

            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);

            D = uv * uv - uu * vv;

            % Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end

            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end

            result = 1;                      % intersectP is in Triangle
        end
        
        %% Function for LinePlaneIntersection
        function [intersectionPoint,check] = myLinePlaneIntersection(self,planeNormal,pointOnPlane,point1OnLine,point2OnLine)
            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;

            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
        
        %% Function to set desired domino path
        function SetDominoPath(self, pathInput)
            % UPDATE REQUIRED
            % - Program does not check for path validity of the line,
            % meaning that it can pass through the robot base.
            % - Function will only work with one robot, and requires
            % modification if intending to implement more robots.
            
            % NOTE - Path inputs are specified from the robots base frame
            % Log the sim state
            self.logObj.LogInfo('[SIM] Setting Domino Path');
            
            % Set the path type
            self.pathType = pathInput;
            
            % Set the domino path based on user input
            % Circle or semicircle path
            if (pathInput == self.CIRCLE || pathInput == self.SEMICIRCLE)
                % Automatically scale radius based on domino number
                radiusArray = [self.DOMINOMIN, self.dominosTotal, self.DOMINOMAX];
                mappedArray = (radiusArray-min(radiusArray))* ...
                    ((self.ROBOTREACH-0.01)-self.ROBOTBASERADIUS)/...
                    (max(radiusArray)-min(radiusArray)) + self.ROBOTBASERADIUS;
                
                % Start point for circle is located on the x axis => y=0
                pathX = mappedArray(2);
                
                % Set start and end point for circle or semicircle
                startPt = transl(pathX,0,0);
                if (pathInput == self.CIRCLE)
                    endPt = startPt;
                else
                    endPt = transl(-1*pathX,0,0);
                end
            
            % Straight Line Path
            elseif (pathInput == self.LINE)
                
                %INCOMPLETE
                
                startPt = transl(0,0,0);
                endPt = tranls(0,0,0);
                
            else %ERROR
                self.logObj.LogInfo('[SIM] ERROR - Domino path not specified');
            end
            
            % Set the start and end pts for the path
            self.pathStartPt = self.objList{self.MYCOBOT}{1}.model.base * startPt;
            self.pathEndPt = self.objList{self.MYCOBOT}{1}.model.base * endPt;
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
            % Set a circle or semicircle path around the robot with the specified startPt
            if (self.pathType == self.CIRCLE || self.pathType == self.SEMICIRCLE)
                % Determine radius as distance from startPt to robot base
                pathRadius = hypot(self.pathStartPt(13),self.pathStartPt(14));
                
                % Set the angle and increment for the circle or semicircle
                if (self.pathType == self.SEMICIRCLE)
                    % Determine distance around semicircle for each domino
                    goalTransIncrement = (pi*pathRadius)/self.dominosTotal;
                    % Semicircle angle
                    angleTotal = 180;
                else
                    % Determine distance around circle for each domino
                    goalTransIncrement = (2*pi*pathRadius)/self.dominosTotal;
                    % Circle total angle
                    angleTotal = 360;
                end
                
                % Determine the angle increment for each domino
                goalAngleIncrement = angleTotal/self.dominosTotal;
                
                % Set the transform for the first domino (in world frame)
                pathTF = self.pathStartPt * transl(0, 0, self.objList{self.DOMINO}{1}.dominoZOffset);
                
                % Add goal poses to dominoes (in world frame)
                for i = 1:self.dominosTotal
                    
                    % Set the goal pose of the current domino
                    self.objList{self.DOMINO}{i}.desiredPose = pathTF;
                    
                    % Determine the goal pose of the next domino
                    pathTF = pathTF * trotz(deg2rad(goalAngleIncrement/2)) * ...
                        transl(0, goalTransIncrement, 0) * ...
                        trotz(deg2rad(goalAngleIncrement/2));
                end
            elseif (self.pathType == self.LINE)
                % INCOMPLETE
            else % ERROR
                self.logObj.LogInfo('[SIM] ERROR - Domino path not set');
            end
        end
    end
end