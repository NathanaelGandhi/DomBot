classdef log4matlab < handle 
    %LOG4MATLAB This is a simple logger written with log4cxx in mind
    
    properties (Constant)
        DEBUG = 0;
        WARN = 1;
        ERROR = 2;
        NONE = 3; % No messages at all
    end
        
    properties(SetObservable = true)
        mlog;
    end
    
    properties(Access = protected)
        logger;          
    end
    
    properties(Access = private)
        listener2mlog;
    end
    
    properties(SetAccess = protected)
        fullpath = 'log4matlab.log';
        append = false;
        commandWindowLevel = log4matlab.NONE; % By default nothing printed to file will be printed to the console. Must be greater or equal to log to file level
    end

    
    methods
%% ...structors
        function self = log4matlab(fullpath_passed,doAppend)           
            if 0 < nargin
                path = fullpath_passed;
                if nargin > 1
                    self.SetAppend(doAppend);
                end
            end
			self.SetFilename(path);
            self.listener2mlog = addlistener(self,'mlog','PostSet',@self.WriteToFile);
        end
        
%% SetFilename        
        function SetFilename(self,fullpath_passed)
            self.fullpath = fullpath_passed;
            if ~self.append
                fid = fopen(self.fullpath,'w');
                fclose(fid);
            end
        end
          
%% SetLoggerLevel  
        function SetLoggerLevel(self,loggerIdentifier,value)%#ok<MANU>
            loggerIdentifier = strrep(loggerIdentifier, '.', '_');
            eval(['self.logger.',loggerIdentifier,'.value = ',num2str(value),';']);
            pause(0);
        end
        
%% GetLoggerLevel        
        function result = GetLoggerLevel(self,loggerIdentifier)
            % Default is debug
            result = self.DEBUG;            
            loggerIdentifier = strrep(loggerIdentifier, '.', '_');
            
            if isfield(self.logger,loggerIdentifier) 
                eval(['result = self.logger.',loggerIdentifier,'.value;']);
            else
                self.SetLoggerLevel(loggerIdentifier,result)
            end                
        end
        
%% SetCommandWindowLevel 
        function SetCommandWindowLevel(self,value)
            self.commandWindowLevel = value;
            pause(0);
        end
    end
       
    methods (Access = private)
%% SetAppend
        function SetAppend(self,value)
            if value
                self.append = true;
            else
                self.append = false;
            end
        end
        
%% WriteToFile        
        function WriteToFile(self,src,evnt)             %#ok<INUSD>
            
            if ~iscell(self.mlog) || size(self.mlog,2)~=3
                error('Problem with the value of mlog that has been set');
            elseif isempty(self.fullpath)
                error('You must set fullpath using SetFilename() method');
            end
            
            loggerIdentifier = strrep(self.mlog{2}, '.', '_');            
            % If there is no field by this name or the logger level set is 
            % less than the level passed then print to file
            DoIt = true;
            if isfield(self.logger,loggerIdentifier) 
                tempField = getfield(self.logger,loggerIdentifier); %#ok<GFLD>
                if self.mlog{1} < tempField.value
                    DoIt = false;
                end
            end
                
            if DoIt
                if (self.mlog{1} == self.DEBUG), levelStr = 'DEBUG';
                elseif (self.mlog{1} == self.WARN), levelStr = 'WARN';
                else levelStr = 'ERROR';
                end

                try                 
                    fid = fopen(self.fullpath,'a');
                    fprintf(fid,'%s %s %s - %s\r\n' ...
                        , datestr(now,'yyyy-mm-dd HH:MM:SS,FFF') ...
                        , levelStr ...
                        , self.mlog{2} ... % Have left this one with the '.' if it is passed
                        , self.mlog{3});
                    fclose(fid);
                catch ME_1
                    display(ME_1);
                end  
				
                % If necessary write to command window (note it will only be written to the conole if: 
                % 1) it is written to file AND
                % 2) the min commandWindowLevel is less than or equal to the log level
                if self.commandWindowLevel <= self.mlog{1}
                    display([self.mlog{2},': ',self.mlog{3}]);                
                end
            end

        end
	end
	
	methods (Static)
		
%% MatrixToString		
		function str = MatrixToString(matrix)
			str = char(10); % Line feed
			for (i=1:size(matrix,1))
				for (j=1:size(matrix,2))
					element = matrix(i,j);
					str = sprintf([str,'% 10.4f   '],element);
				end
				str = [str,char(10)]; %#ok<AGROW>
			end		
		end
		
%% ExceptionToString
		function str = ExceptionToString(ME)
			str = char(10); % Line feed			
			str = [str,'ME.identifier: ',ME.identifier,char(10)];
			str = [str,'ME.message: ',ME.message,char(10)];
			for i=1:length(ME.stack)
				str = [str,'ME.stack: ',ME.stack(i).name,', Line ',num2str(ME.stack(i).line),char(10)]; %#ok<AGROW>
			end
        end
        
%% Me
%> Returns the current function name
        function str = Me()
            dbstackOutput = dbstack;
            if size(dbstackOutput,1) < 2
                str = '';
                return 
            end
            fullName = dbstackOutput(end).name;
            strReversed = strtok(fullName(end:-1:1),'.');
            str = [strReversed(end:-1:1),': '];
        end
    end	
end

