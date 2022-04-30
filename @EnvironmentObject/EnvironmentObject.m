classdef EnvironmentObject
    properties
        logObj;     % Log object
        envObj;     % Environment object
        type;       % Environment object type
        pose;       % Environment object Pose
    end
    
    methods%% Class for Table simulation
        function self = EnvironmentObject(logArg, pose, type)
            self.logObj = logArg;   % Store log object
            % Store type param
            if nargin < 3           % Check number of function input arguments
                self.type = 'generic' ;
            else
                self.type = type ;
            end
            self.pose = pose;       % Store Pose param
            self.logObj.LogDebug(['[TBL] Environment Object constructor: ',type]);
        end
    end
end