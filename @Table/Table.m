classdef Table < EnvironmentObject
    properties
        model;  % Class object
        % Vertex count
        tableVertexCount;
    end
    
    methods%% Class for Table simulation
        function self = Table(logArg, pose)
            % Call superclass constructor
            self = self@EnvironmentObject(logArg, pose, 'table');
            self.DrawTable();
        end
        
        % Function to draw the table
        function DrawTable(self)
            % I'm using this method as 'PlaceObject' can't rotate the table
            % Read ply file
            [tris,verts,data] = plyread('Table.ply','tri');  
            % Set vertex count for use in transforms
            self.tableVertexCount = size(verts, 1);
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Update vertexes for new pose
            updatedPoints = [self.pose * [verts,ones(self.tableVertexCount,1)]']';
            % Then plot the trisurf with verticies
            self.model = trisurf(tris,updatedPoints(:,1), updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
end