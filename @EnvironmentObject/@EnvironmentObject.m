classdef EnvironmentObject < handle
    properties
        envObj;  % Handle
        logObj;  % Log object
        type;   % Environment object type
        id;     % Environment object id
        pose;   % Environment object Pose
    end
    
    methods%% Class for Table simulation
        function self = EnvironmentObject(logArg, type, pose)
            self.logObj = logArg;   % Store log object
            self.logObj.LogInfo('[TBL] Table constructor');
            % Store type param
            if nargin < 3           % Check number of function input arguments
                obj.type = 'generic' ;
            else
                obj.type = type ;
            end
            self.pose = pose;       % Store Pose param
        end
    end
end