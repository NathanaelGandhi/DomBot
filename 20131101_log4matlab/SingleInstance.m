classdef (Sealed) SingleInstance < handle
    % SINGLEINSTANCE this is based directly on the singleinstance class
    % from matlab documentation. Look up singleinstance for more details
    
    properties (Access = protected)
        % This is the general value that will be stored for whatever the 
        value;
        requiredType;
    end
    methods (Access = private)
        function self = SingleInstance(requiredType_passed)
            self.requiredType = requiredType_passed;
        end
        
%% testAndSet: Handles the general input
        function testAndSet(self,varagin)
            if isa(varagin,self.requiredType)
                self.value = varagin;
            else
                whosOutput = whos('varagin');
                warning(['Expecting varagin to be of type: ',self.requiredType ...
                        ,'. However, input passed in is of type: ',whosOutput.class]); %#ok<WNTAG>
            end               
        end        
    end
    
    methods (Static)
		
%% Logger 
        function result = Logger(varagin)
            persistent localObj;
            if isempty(localObj) || ~isvalid(localObj)
                localObj = SingleInstance('log4matlab'); 
                localObj.value = log4matlab('logFileName2.log');
            end
            
            if nargin > 0
                localObj.testAndSet(varagin);
            end
            result = localObj.value;
		end	        
    end
end