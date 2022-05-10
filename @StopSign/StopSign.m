classdef StopSign < EnvironmentObject
    properties
        model;                      % Class object
        vertexCount;    % Vertex count
    end
    
    methods%% Class for Stop Sign simulation
        function self = StopSign(logArg, id, pose)
            % Call superclass constructor
            self = self@EnvironmentObject(logArg, id, pose, 'stopSign');
            
            % Read ply file
            [tris,verts,data] = plyread('StopSign_0.002.ply','tri');
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
    end
end