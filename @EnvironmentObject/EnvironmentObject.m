classdef EnvironmentObject
    properties
        logObj;     % Log object
        type;       % Environment object Type
        id;         % Environment object Id
        pose;       % Environment object Pose
    end
    
    methods%% Class for Table simulation
        % Constructor
        function self = EnvironmentObject(logArg, id, pose, type)
            self.logObj = logArg;   % Store log object
            % Store type param
            if nargin < 3           % Check number of function input arguments
                self.type = 'generic' ;
            else
                self.type = type ;
            end
            self.id = id;           % Store Id param
            self.pose = pose;       % Store Pose param
            self.logObj.LogInfo(['[TBL] Environment Object constructor: ',self.type,':',num2str(self.id)]);
        end
    end
end