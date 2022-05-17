classdef Person < EnvironmentObject
    properties
        model;          % Class object
        to_h;           % Teach Object handle
        vertexCount;    % Vertex count
    end
    
    methods%% Class for Person simulation
        function self = Person(logArg, id, pose)
            % Call superclass constructor
            self = self@EnvironmentObject(logArg, id, pose, 'person');
            
            % Read ply file
            [tris,verts,data] = plyread('full_body0.001.ply','tri');
            % Set vertex count for use in transforms
            self.vertexCount = size(verts, 1);
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            try
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            catch
                % Ply data var uses xyz, not rgb
                vertexColours = [data.vertex.x, data.vertex.y, data.vertex.z] / 255;
            end
            % Set desired base pose
            self.pose = pose;
            % Update vertexes for new pose
            updatedPoints = [self.pose * [verts,ones(self.vertexCount,1)]']';
            % Plot the trisurf with verticies
            self.model = trisurf(tris,updatedPoints(:,1), updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
                
        %% Function to start "teach person"
        function StartPersonTeach(self)
            self.teachPerson();
        end
        
        %% Update pose function
        function UpdatePose(self, pose)
            % Multiplies the vertices by the inverted transfrom 
            updatedPoints = [self.pose \ [self.model.Vertices,ones(self.vertexCount,1)]']';  
            % Sets pose to new pose
            self.pose = pose;
            % Transforming vertices to new pose location
            updatedPoints = [self.pose * [updatedPoints(:,1:3),ones(self.vertexCount,1)]']'; 
            % Sets new vertex poses
            self.model.Vertices = updatedPoints(:,1:3);
        end 
    end
end