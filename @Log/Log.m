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
    end
    % Const Vars
    properties(Constant)

    end
    
    methods
        % Constructor
        function self = Log()
            LogInfo(self,'[LOG] Log constructor');
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
    end
end