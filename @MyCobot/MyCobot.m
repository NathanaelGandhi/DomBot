classdef MyCobot < EnvironmentObject
    %% Properties
    properties
        % Handles
        model;  % MyCobot handle
        tc;     % Teach Cartesian handle
        
        % Vars
        workspace;
        rangeOfMotionPlot;
        qCurrent = [0, 0, -pi/5, -pi/2 + pi/5, -pi/2, 0];  % Current joint angles
        %qCurrent = [0, 0, 0, -pi/2, -pi/2, 0];  % Current joint angles
        qMatrix;                                % Array of joint angles
        cameraPoints;                      % Array of generated camera points 3xN (N = number of points)
        imagePoints;                       % Array of observed camera points that are projected on an image plane 2xN
        cameraObject;                      % Camera object created from CentralCamera class
        cameraDepth;                       % Distance from cameraObject to cameraPoints (can be array or average distance)
        % Flag used for robotRetreat function
        searchOrRetreatFlag = false;               % 'false' for searching, 'true' for retreating
    end
    
    % Const Vars
    properties(Constant)
        RADIUS_REACH = 0.286;           %280 mm range of motion from MyCobot manual 
        DELTA_T = 0.05;                 % Discrete time step - Calculating trajectory (RMRC)
        W = diag([1 1 1 1 0.1 0.1]);  % Weighting matrix for the velocity vector - Calculating trajectory (RMRC)
        EPSILON = 0.1;                  % Damped Least Squares variables
        LAMBDA_MAX = 5E-2;              % Damped Least Squares variables
        % Set array of points to be attached to stopSignObject 3xN
        % where N = number of points
        SQUARE_OF_POINTS = [0,     0,     0,     0; ...
                         0.02, -0.02, -0.02,  0.02; ...
                         0.02,  0.02, -0.02, -0.02];
        % Ideal image points that robotRetreat uses for visual servoing
        IDEAL_IMAGE_POINTS = [662 362 362 662; 362 362 662 662];
        
        % Image based visual servoing gain value for robotRetreat()
        LAMBDA_GAIN = 0.6;                      
    end
    
    
    %% Static Methods
    methods (Static) 
        %% Calculates distance between transforms
        function distance = disTr(tr1, tr2)
            sq = tr2(1:3, 4)-tr1(1:3, 4);
            distance = sqrt(transpose(sq)*sq);
        end
        
        %% Create MyCobot SerialLink model
        function model = GetMyCobotRobot()
            pause(0.001);
            name = ['MyCobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
            L(1) = Link('d', 0.13156, 'a', 0, 'alpha', pi/2);
            L(2) = Link('d', -0.06639, 'a', 0.1104, 'alpha', 0);
            L(3) = Link('d', 0.06639, 'a', 0.096, 'alpha', 0);
            L(4) = Link('d', -0.06639, 'a', 0, 'alpha', -pi/2);
            L(5) = Link('d', 0.07318, 'a', 0, 'alpha', -pi/2);
            L(6) = Link('d', -0.0436, 'a', 0, 'alpha', 0);
            
            %offset
            L(2).offset = pi/2;
            L(4).offset = -pi/2;
            L(5).offset = pi;
            
            %limits
            L(1).qlim = (pi/180)*[-200 200];
            L(2).qlim = (pi/180)*[-165 165];
            L(3).qlim = (pi/180)*[-165 165];
            L(4).qlim = (pi/180)*[-165 165];
            L(5).qlim = (pi/180)*[-165 165];
            L(6).qlim = (pi/180)*[-175 175];
            
            model = SerialLink(L, 'name', name);
        end
        
    end
    %% Methods    
    methods
        % Constructor
        function self = MyCobot(logArg, id, pose)
            % Call superclass constructor
            self = self@EnvironmentObject(logArg, id, pose, 'mycobot');
            self.workspace = self.SetMyCobotWorkspace();
            self.model = self.GetMyCobotRobot();
            self.cameraObject = self.GetCamera();
            self.PlotAndColourRobot();                      % robot,workspace);
        end
        
        %% Set the workspace to the robot pose + MyCobot offsets
        function workspace = SetMyCobotWorkspace(self)
            workspaceOffset = [-0.5 0.5 -0.5 0.5 0 1];      % Where did these offsets come from?
            % below could be done better with loops
            workspace = [...
                (self.pose(13)+workspaceOffset(1)) (self.pose(13)+workspaceOffset(2)) ...   % x-axis
                (self.pose(14)+workspaceOffset(3)) (self.pose(14)+workspaceOffset(4)) ...   % y-axis
                (self.pose(15)+workspaceOffset(5)) (self.pose(15)+workspaceOffset(6))];     % z-axis
        end
        
        %% To make the robot retreat from a simulated safety symbol using visual servoing and RMRC
        function robotRetreat(self, stopSignObject)
            % Updates stop sign points
            self.generateStopSignPoints(stopSignObject);
            
            % Updates image plane plot
            self.displayImagePlane();
            
            % Checks if the stop sign points are still visible in the image
            % plane
            if isnan(self.imagePoints)
                % If there are any stop sign points that aren't visible,
                % the robot would have to switch to searchForStopSign mode
                self.searchOrRetreatFlag = false;
                return;
            end
            
            % Calculate the error between the ideal camera image points and
            % the projected camera image points
            imageError = self.IDEAL_IMAGE_POINTS - self.imagePoints;
            
            % Changes this array from 2x4 into 8x1
            imageError = imageError(:);
            
            % Generates visual motion jacobian
            visualJacobian = cam.visjac_p(self.imagePoints, self.cameraDepth);
            
            cameraVelocity = self.LAMBDA_GAIN * pinv(visualJacobian) * imageError;
            
            % Calculate joint and inverse joint jacobian
            jointJacobian = self.model.jacobn(self.qCurrent);
            jointJInverse = pinv(jointJacobian);
            
            % Calculate qDot (joint velocities)
            qDot = jointJInverse*cameraVelocity;
            
            % Clear qMatrix
            self.qMatrix = zeros(1,6);
            
            % Set qMatrix
            self.qMatrix(1,:) = self.qCurrent + self.DELTA_T*qDot;
            
            self.RunTraj();
            
        end
        
        %% Makes robot search for stop sign points using cameraObject
        function searchForStopSign(self, stopSignObject)
            % Updates position of stopSignObject square points
            self.generateStopSignPoints(stopSignObject);
            % Updates camera image plane plot and changes imagePoints
            self.displayImagePlane();
            
            % When the displayImagePlane function can't display a
            % cameraPoint on its image plane plot, it would change the
            % points values (uv) on imagePoints to NaN
            if isnan(self.imagePoints)
            else
                % If all of the imagePoints are real number values, then
                % the image plane is correctly viewing the entire set of
                % cameraPoints and should switch modes to the robotRetreat
                % function.
                self.searchOrRetreatFlag = true;
                return;
            end
            
        end
        %% Generate points on a StopSign to allow MyCobot to detect with cameraObject
        % In order for cameraObject to detect a StopSign object during 
        % robotRetreat, the stop sign needs to have a list of points 
        % to be passed through the cameraObject functions.
        %
        % Use this function to update cameraPoints whenever the
        % stopSignObject changes it's position
        %
        % self = MyCobot object
        % stopSignObject = StopSign Object
        % cameraPoints = Array of stop sign points this function generates
        
        function generateStopSignPoints(self, stopSignObject)
            % Clears cameraPoints every time this function runs.
            % Clearing cameraPoints is important in order to update the
            % stopSignObject's points when it changes poition.
            self.cameraPoints = zeros(3,4);
            
            % Calculate transform for center of stop sign
            % Currently, 0.42 is the estimated height to the stop signs
            % center.
            % Also rotates in y axis just for the square points to be
            % aligned properly.
            stopSignCenterT = stopSignObject.pose*transl(0,0.42,0)*troty(pi/2);
            
            % squareOfTransforms is an array of transforms, where each
            % transform represents a point from squareOfPoints.
            squareOfTransforms = zeros(4,4,4);
            
            % Clears cameraDepth because it needs to get updated every time
            % cameraPoints are generated/updated
            self.cameraDepth = zeros(1,6);
            
            % Sets cameraPoint array
            for i=1:4
                
                % Maps squareOfPoints to the stopSignObject position
                squareOfTransforms(:,:,i) = stopSignCenterT*transl(transpose(self.SQUARE_OF_POINTS(:,i)));
                
                % Sets cameraPoints as the position valuse from
                % squareOfTransfoms
                self.cameraPoints(:,i) = squareOfTransforms(1:3, 4, i);
                
                % plots cameraPoints onto stop sign
                % plot3(self.cameraPoints(1,i),self.cameraPoints(2,i),self.cameraPoints(3,i),'o','Color','g');
                
                % Sets cameraDepth as the distance between cameraObject and
                % the cameraPoints 
                self.cameraDepth(i,1) = self.disTr(self.cameraObject.T, squareOfTransforms(:,:,i));
            end
        end
        
        %% Display plot of image plane for visual servoing
        % Creates plot for cameraObject image plane.
        % cameraPoints should be set before this function runs.
        % Returns imagePoints, which are 2xN arrays of points from the image
        % plane that represent the points the cameraObjejct sees
        function displayImagePlane(self)
            % Checks if cameraPoints are empty
            if isempty(self.cameraPoints)
                error('No cameraPoints were generated before displayImagePlane ran');
            else
                % Clears cameraObject plot so it can be updated
                self.cameraObject.clf;
                
                % Plots ideal image points
                self.cameraObject.plot(self.IDEAL_IMAGE_POINTS,'+', 'Color', 'r');
                
                % Holds image plane plot to allow for the second plot
                % data to be undated.
                self.cameraObject.hold(true)
                
                % Plots the cameraPoints onto the cameraObject image plane.
                % If the cameraPoints are not in cameraObject's field of
                % view, then the non-visable points won't show up on the
                % image plane plot.
                self.imagePoints = self.cameraObject.plot(self.cameraPoints, '.', 'Color', 'k', 'MarkerSize', 20);
            end
        end
        
        %% Create camera for visual servoing
        function cameraObject = GetCamera(self)
            cameraObject = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
            'resolution', [1024 1024], 'centre', [512 512],'name', 'MyCobotCamera');
            cameraObject.T = self.model.fkine(self.qCurrent)*trotx(pi);
        end
        
        
        %% PlotAndColourRobot
        function PlotAndColourRobot(self)
            % Generate face, vertex and ply data for all links
            self.logObj.LogDebug('[MyCobot] Generate face, vertex and ply data for all links');
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['MyCobot_Links/MyCobotLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.logObj.LogDebug('[MyCobot] Display robot');
            self.model.base = self.pose;
            self.model.plot3d(self.qCurrent,'noarrow','notiles');
            %self.model.plot3d(self.qCurrent,'noarrow','workspace',self.workspace);  % Use this to enable tiles
            hold on
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            self.logObj.LogDebug('[MyCobot] Try to colour the robot');
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        %% Range of motion functions
        function baseRange(self)
            zz = 0.13156 + self.model.base(3, 4);
            for i = -165:10: 165
                for j = 0:0.05:self.RADIUS_REACH
                    xx = j*sin(i*pi/180)+ self.model.base(1, 4);
                    yy = j*cos(i*pi/180) + self.model.base(2, 4);
                    self.rangeOfMotionPlot = plot3(xx,yy,zz, '.','Color','g','MarkerSize',10);
                end
            end
        end
        
        function fullRange(self)
            for i = -180:10: 180
                for j = 0:10:70
                    xx = self.RADIUS_REACH*sin(i*pi/180)*cos(j*pi/180)+ self.model.base(1, 4);
                    yy = self.RADIUS_REACH*cos(i*pi/180)*cos(j*pi/180)+ self.model.base(2, 4);
                    zz = self.RADIUS_REACH*sin(j*pi/180) + 0.13156 + self.model.base(3, 4);
                    plot3(xx,yy,zz, '.','Color','b','MarkerSize',10);
                end
            end
        end   
        
        
        %% calculates jtraj (with RMRC)
        function CalculateTraj(self, Transform, steps)
        % CalculateTraj
        % Incorporates RMRC and damped least squares
        % Default end effector trajectory is a straight line, Ti to Tf
        % checks if inputed transform exceeds range of motion (280 mm)
        % 1.2) Allocate array data
        self.qMatrix = zeros(steps,6);          % Array for joint angles
        qdot = zeros(steps,6);             % Array for joint velocities
        theta = zeros(3,steps);            % Array for roll-pitch-yaw angles
        x = zeros(3,steps);                % Array for x-y-z trajectory
        
        % Calculates trapizoidal trajectory of end effector position
        Ti = self.myFkine(self.qCurrent);   % Transform of current end effector position
        Tf = Transform;                         % Transform of final end effector position
        if steps == 1
            s = 1;
        else
            s = lspb(0,1,steps);               % Trapezoidal trajectory scalar
        end
        for i=1:steps
            x(:,i) = (1-s(i))*Ti(1:3, 4)+s(i)*Tf(1:3,4);
            
           
            % End effector rotation changes to inputted tramsform
            theta(:,i) = (1-s(i))*tr2rpy(Ti)+s(i)*tr2rpy(Tf);            
        end
        % Sets first step of qMatrix
        TFirstStep = rpy2tr(theta(:,1)');
        TFirstStep(1:3,4) = x(:,1);
        self.qMatrix(1,:) = self.model.ikcon(TFirstStep,self.qCurrent);
        
        for i=1:steps-1
            T = self.myFkine(self.qMatrix(i,:));
            deltaX = x(:,i+1) - T(1:3, 4);      % Get position error from next waypoint
            Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
            Ra = T(1:3, 1:3);                % Current end-effector rotation matrix

            Rdot = (1/self.DELTA_T)*(Rd - Ra);       % Calculate rotation matrix error (see RMRC lectures)
            S = Rdot*Ra;                      % Skew symmetric! S(\omega)
            linear_velocity = (1/self.DELTA_T)*deltaX;
            angular_velocity = [S(3,2);S(1,3);S(2,1)];  % Check the structure of Skew Symmetric matrix! Extract the angular velocities. (see RMRC lectures)
            xdot = self.W*[linear_velocity; angular_velocity];              % Calculate end-effector velocity to reach next waypoint.
            J = self.model.jacob0(self.qMatrix(i,:));                 % Get Jacobian at current joint state
            mu = sqrt(det(J*J'));
            if mu < self.EPSILON  % If manipulability is less than given threshold
                lambda = (1-(mu/self.EPSILON)^2)*self.LAMBDA_MAX; % Damping coefficient (try scaling it)
            else
                lambda = 0;
            end
            invJ = inv(J'*J+lambda*eye(6))*J'; % Apply Damped Least Squares pseudoinverse
            qdot(i,:) = (invJ*xdot)'; % Solve the RMRC equation (you may need to transpose the         vector)
            for j = 1:6 % Loop through joints 1 to 6
                if qdot(i,j)*self.DELTA_T < self.model.qlim(j,1)% If next joint angle is lower than joint limit...
                    qdot(i,j) = 0; % Stop the motor
                elseif qdot(i,j)*self.DELTA_T > self.model.qlim(j,2) % If next joint angle is greater than joint limit ...
                    qdot(i,j) = 0; % Stop the motor
                end
            end
            self.qMatrix(i+1,:) = self.qMatrix(i,:) + self.DELTA_T*qdot(i,:); % Update next joint state based on joint velocities
        end
        end
        %% calculate quintic polynomial trajectory (to work with RunTraj)
        function JTraJ(self, qGoal, steps)
            self.qMatrix = jtraj(self.model.getpos, qGoal, steps);
            
            
        end
        %% runs trajectory (with RMRC)
        function RunTraj(self)
            % Should be placed in a for loop with the same number of steps
            % that was calculated
            self.model.animate(self.qMatrix(1,:));
            self.qCurrent = self.qMatrix(1,:);
            self.cameraObject.T = self.model.fkine(self.qCurrent)*trotx(pi);
            self.qMatrix(1,:) = [];
            drawnow
        end
        
        %% Sim RobotRetreat function
        function RobotRetreat(self)
            if self.searchOrRetreatFlag
                self.robotRetreat(stopSignObject);
            else
                self.searchForStopSign(stopSignObject);
            end
        end
        
        %% Function to start "teach classic"
        function StartClassicTeach(self)
            self.teach();
        end
        
        %% Function to start "teach cartesian"
        function StartCartesianTeach(self)
            self.teachCartesian();
        end
    end
end