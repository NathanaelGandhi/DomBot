classdef Extinguisher < Log
    properties
        % Handle
        model;
        
        % Position of fire extinguisher
        pose;
        
        % Vertex count
        extinguisherVertexCount;
    end
    
    methods%% Class for Fire Extinguisher simulation
        function self = Extinguisher(pose)
            self@Log();     % Superclass Log
            % Read ply file
            [tris,verts,data] = plyread('Extinguisher.ply','tri');
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