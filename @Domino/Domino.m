classdef Domino < handle
    properties
        % Handle
        model;
        
        % Current pose of domino
        pose;
        
        % Final pose
        desiredPose;
        
        % Domino vertex count
        vertexCount;
        
        % State of Domino
        domninoState;
    end
    
    methods %% Class for Domino simulation
        function self = Domino(pose)
            % Read ply file
            [tris,verts,data] = plyread('Domino.ply','tri');
            % Set vertex count to use in transforms
            self.vertexCount = size(verts, 1);
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Setting base pose
            self.pose = pose;
            updatedPoints = [self.pose * [verts,ones(self.vertexCount,1)]']';  
            % Then plot the trisurf with verticies
            self.model = trisurf(tris,updatedPoints(:,1), updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
        
        function UpdatePose(self, pose)
            % Multiplies the vertices by the inverted domino transfrom 
            updatedPoints = [self.pose \ [self.model.Vertices,ones(self.vertexCount,1)]']';  
            % Sets domino pose to new pose
            self.pose = pose;
            % Transfroming vertices to new pose location
            updatedPoints = [self.pose * [updatedPoints(:,1:3),ones(self.vertexCount,1)]']'; 
            % Sets new domino vertex poses
            self.model.Vertices = updatedPoints(:,1:3);
        end 
    end
end