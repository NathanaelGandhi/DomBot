classdef Log < handle
    % Vars
    properties
        log;     % Handle
        debug;       % Flag for Log Debug
        info;        % Flag for Log Info
        warn;        % Flag for Log Warn
        error;       % Flag for Log Error
        fatal;       % Flag for Log Fatal
        log2File;    % Flag for Log to File
        fileLoggerObj; % log4matlab - Gavin (2022). log4matlab (https://www.mathworks.com/matlabcentral/fileexchange/33532-log4matlab), MATLAB Central File Exchange. Retrieved May 3, 2022.
    end
    % Const Vars
    properties(Constant)

    end
    
    methods
        % Constructor
        function self = Log()
            LogInfo(self,'[LOG] Log constructor');
            % Default all logging off
            self.debug = false;
            self.info = false;
            self.warn = false;
            self.error = false;
            self.fatal = false;
            self.log2File = false;
        end
        %Deconstructor
        function delete(self)
            LogInfo(self,'[LOG] Log deconstructor');
        end
        
        % Set Log Levels
        function SetDebugFlag(self,flag)
            self.debug = flag;
        end
        function SetInfoFlag(self,flag)
            self.info = flag;
        end
        function SetWarnFlag(self,flag)
            self.warn = flag;
        end
        function SetErrorFlag(self,flag)
            self.error = flag;
        end
        function SetFatalFlag(self,flag)
            self.fatal = flag;
        end
        function SetLog2FileFlag(self,flag)
            self.log2File = flag;
        end
        
        % Print Log
        function LogDebug(self,txt)
            if(self.debug)
                logOutput = [datestr(now,'HH:MM:SS'),' DEBUG: ',txt];
                disp(logOutput);
            end
        end
        function LogInfo(self,txt)
            if(self.info)
                logOutput = [datestr(now,'HH:MM:SS'),' INFO: ',txt];
                disp(logOutput);
            end
        end
        function LogWarn(self,txt)
            if(self.warn)
                logOutput = [datestr(now,'HH:MM:SS'),' WARN: ',txt];
                disp(logOutput);
            end
        end
        function LogError(self,txt)
            if(self.error)
                logOutput = [datestr(now,'HH:MM:SS'),' ERROR: ',txt];
                disp(logOutput);
            end
        end
        function LogFatal(self,txt)
            if(self.fatal)
                logOutput = [datestr(now,'HH:MM:SS'),' FATAL: ',txt];
                disp(logOutput);
            end
        end
        function Log2File(self, level, txt)
            if(self.log2File)
                self.fileLoggerObj.mlog = {level,'',txt};
            end
        end
    end
end