classdef EnvironmentObject < handle
    properties
        envObj;  % Handle
        logObj;  % Log object
        type;   % Environment object type
        id;     % Environment object id
        pose;   % Environment object Pose
    end
    
    methods%% Class for Table simulation
        function self = Table(logArg, type, id, pose)
            self.logObj = logArg;   % Store log object
            self.logObj.LogInfo('[TBL] Table constructor');
            % Store type param
            if nargin < 4           % Check number of function input arguments
                obj.type = 'generic' ;
            else
                obj.type = type ;
            end
            self.id = id;           % Store id param
            self.pose = pose;       % Store Pose param
        end
    end
end