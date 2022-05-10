classdef MyCobot < EnvironmentObject
    %% Properties
    properties
        model;  % Class object
        workspace;
        radiusOfMotion = 0.28; %280 mm range of motion from MyCobot manual 
        rangeOfMotionPlot;
        
        % Variables for calculating trajectory (RMRC)
        qCurrent = [0, 0, 0, -pi/2, -pi/2, 0];  % Current joint angles
        qMatrix;                                % Array of joint angles
        steps;                              
        deltaT = 0.05;                          % Discrete time step
        W = diag([1 1 1 0.1 0.1 0.1]);          % Weighting matrix for the velocity vector
        
        % Damped Least Squares variables
        epsilon = 0.1;
        lambdaMax = 5E-2;
    end
    
    
    %% Static Methods
    methods (Static) 
        %calculates distance between transforms
        function distance = disTr(tr1, tr2)
            sq = tr2(1:3, 4)-tr1(1:3, 4);
            distance = sqrt(transpose(sq)*sq);
        end
        
        function model = GetMyCobotRobot()
            pause(0.001);
            name = ['MyCobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
            L(1) = Link('d', 0.13156, 'a', 0, 'alpha', pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
            L(2) = Link('d', -0.06639, 'a', 0.1104, 'alpha', 0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L(3) = Link('d', 0.06639, 'a', 0.096, 'alpha', 0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L(4) = Link('d', -0.06639, 'a', 0, 'alpha', -pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
            L(5) = Link('d', 0.07318, 'a', 0, 'alpha', -pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
            L(6) = Link('d', -0.0436, 'a', 0, 'alpha', 0,'qlim',[deg2rad(-360),deg2rad(360)]);
            
            %offset
            L(2).offset = pi/2;
            L(4).offset = -pi/2;
            L(5).offset = pi;
            
            %limits
            L(1).qlim = (pi/180)*[-165 165];
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
        function robotRetreat(self)
            
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
            self.model.plot3d(self.qCurrent,'noarrow','workspace',self.workspace);
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
                for j = 0:0.05:self.radiusOfMotion
                    xx = j*sin(i*pi/180)+ self.model.base(1, 4);
                    yy = j*cos(i*pi/180) + self.model.base(2, 4);
                    self.rangeOfMotionPlot = plot3(xx,yy,zz, '.','Color','g','MarkerSize',10);
                end
            end
        end
        
        function fullRange(self)
            for i = -180:10: 180
                for j = 0:10:70
                    xx = self.radiusOfMotion*sin(i*pi/180)*cos(j*pi/180)+ self.model.base(1, 4);
                    yy = self.radiusOfMotion*cos(i*pi/180)*cos(j*pi/180)+ self.model.base(2, 4);
                    zz = self.radiusOfMotion*sin(j*pi/180) + 0.13156 + self.model.base(3, 4);
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
        self.steps = steps;
        self.qMatrix = zeros(self.steps,6);          % Array for joint angles
        qdot = zeros(self.steps,6);             % Array for joint velocities
        theta = zeros(3,self.steps);            % Array for roll-pitch-yaw angles
        x = zeros(3,self.steps);                % Array for x-y-z trajectory
        positionError = zeros(3,self.steps);    % For plotting trajectory error
        angleError = zeros(3,self.steps);       % For plotting trajectory error
        
        % Calculates trapizoidal trajectory of end effector position
        Ti = self.model.fkine(self.qCurrent);   % Transform of current end effector position
        Tf = Transform;                         % Transform of final end effector position
        s = lspb(0,1,self.steps);               % Trapezoidal trajectory scalar
        for i=1:self.steps
            x(:,i) = (1-s(i))*Ti(1:3, 4)+s(i)*Tf(1:3,4);
            %theta(:,i) = (1-s(i))*tr2rpy(Ti)+s(i)*tr2rpy(Tf);
            theta(:,i) = tr2rpy(trotx(0));
        end
        self.qMatrix(1,:) = self.model.ikcon(Tf,self.qCurrent);
        for i=1:self.steps-1
            T = self.model.fkine(self.qMatrix(i,:));
            deltaX = x(:,i+1) - T(1:3, 4);      % Get position error from next waypoint
            Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
            Ra = T(1:3, 1:3);                % Current end-effector rotation matrix

            Rdot = (1/self.deltaT)*(Rd - Ra);       % Calculate rotation matrix error (see RMRC lectures)
            S = Rdot*Ra;                      % Skew symmetric! S(\omega)
            linear_velocity = (1/self.deltaT)*deltaX;
            angular_velocity = [S(3,2);S(1,3);S(2,1)];  % Check the structure of Skew Symmetric matrix! Extract the angular velocities. (see RMRC lectures)
            deltaTheta = tr2rpy(Rd*Ra);% Convert rotation matrix to RPY angles
            xdot = self.W*[linear_velocity; angular_velocity];              % Calculate end-effector velocity to reach next waypoint.
            J = self.model.jacob0(self.qMatrix(i,:));                 % Get Jacobian at current joint state

            mu = sqrt(det(J*J'));
            if mu < self.epsilon  % If manipulability is less than given threshold
                lambda = (1-(mu/self.epsilon)^2)*self.lambdaMax; % Damping coefficient (try scaling it)
            else
                lambda = 0;
            end
            invJ = inv(J'*J+lambda*eye(6))*J'; % Apply Damped Least Squares pseudoinverse
            qdot(i,:) = (invJ*xdot)'; % Solve the RMRC equation (you may need to transpose the         vector)
            for j = 1:6 % Loop through joints 1 to 6
                if qdot(i,j)*self.deltaT < self.model.qlim(j,1)% If next joint angle is lower than joint limit...
                    qdot(i,j) = 0; % Stop the motor
                elseif qdot(i,j)*self.deltaT > self.model.qlim(j,2) % If next joint angle is greater than joint limit ...
                    qdot(i,j) = 0; % Stop the motor
                end
            end
            self.qMatrix(i+1,:) = self.qMatrix(i,:) + self.deltaT*qdot(i,:); % Update next joint state based on joint velocities
            positionError(:,i) = deltaX;  % For plotting
            angleError(:,i) = deltaTheta; % For plotting
        end
        end
        
        %% runs trajectory (with RMRC)
        function RunTraj(self, increment)
            % Should be placed in a for loop with the same number of steps
            % that was calculated
            self.model.animate(self.qMatrix(increment,:));
            self.qCurrent =  self.qMatrix(increment,:);
            drawnow;
        end
        
        %% Function to start "teach"
        function StartTeach(self)
            self.teach();
        end
    end
end