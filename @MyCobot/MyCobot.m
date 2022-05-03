classdef MyCobot < EnvironmentObject
    %% Properties
    properties
        model;  % Class object
        workspace;
        radiusOfMotion = 0.28; %280 mm range of motion from MyCobot manual 
        rangeOfMotionPlot;
        
        %variables for calculating trajectory
        q1 = zeros(1,6);
        q2;
        Traj = 0;
        steps = 0;
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
        
<<<<<<< HEAD
        %% Set the workspace to the robot pose + MyCobot offsets
=======
        % Set the workspace to the robot pose + MyCobot offsets
>>>>>>> aace24fa4f9ac41a19a98258ebdec964d197694b
        function workspace = SetMyCobotWorkspace(self)
            workspaceOffset = [-0.5 0.5 -0.5 0.5 0 1];      % Where did these offsets come from?
            % below could be done better with loops
            workspace = [...
                (self.pose(13)+workspaceOffset(1)) (self.pose(13)+workspaceOffset(2)) ...   % x-axis
                (self.pose(14)+workspaceOffset(3)) (self.pose(14)+workspaceOffset(4)) ...   % y-axis
                (self.pose(15)+workspaceOffset(5)) (self.pose(15)+workspaceOffset(6))];     % z-axis
        end

        %% PlotAndColourRobot
        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n
                
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['MyCobot_Links/MyCobotLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(self.q1,'noarrow','workspace',self.workspace);
            hold on
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
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
        
        
        
        %% calculates jtraj
        function Trajectory = calculateTraj(self, Transform, steps)
            %checks if inputed transform exceeds range of motion (280 mm)
            if (MyCobot.disTr(self.model.base()*transl(0,0, 0.13156), Transform) > 0.28)
                disp('MyCobot_ERROR: Transform exceeds MyCobots Range of Motion');
                disp('Possible_Solution: Input a transform that is within the 0.28m radius of your MyCobot');
            else
                self.steps = steps;
                self.q1 = self.model.getpos;
                self.q2 = self.model.ikine(Transform, self.q1, [1 1 1 1 1 0]);
                self.Traj = jtraj(self.q1, self.q2, self.steps);
                Trajectory = self.Traj;
            end
        end
        
        %% runs jtraj calculation 
        function self = runTraj(self)
            if self.steps > 0            
                for i = 1:self.steps
                    self.model.animate(self.Traj(i,:));
                    drawnow();
                end
                self.steps = 0;
            else
                disp('MyCobot_ERROR: You have not calculated your trajectory yet');
                disp('Possible_Solution: Use "calculateTraj(self, transform, steps)" before using "runTraj(self)" ');
            end
           
        end
    end
end