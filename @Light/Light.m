classdef Light < EnvironmentObject
    properties
        model;                      % Class object
        extinguisherVertexCount;    % Vertex count
    end
    
    methods%% Class for Fire Light simulation
        function self = Light(logArg, id, pose)
            % Call superclass constructor
            self = self@EnvironmentObject(logArg, id, pose, 'light');
            
            % Read ply file
            [tris,verts,data] = plyread('Light.ply','tri');
            % Set vertex count for use in transforms
            self.extinguisherVertexCount = size(verts, 1);
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Set desired base pose
            self.pose = pose;
            % Update vertexes for new pose
            updatedPoints = [self.pose * [verts,ones(self.extinguisherVertexCount,1)]']';
            % Plot the trisurf with verticies
            self.model = trisurf(tris,updatedPoints(:,1), updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
end