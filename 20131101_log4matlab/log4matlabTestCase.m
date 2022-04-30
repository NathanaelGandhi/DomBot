classdef log4matlabTestCase< TestCase
    methods

%% test_construct
        function test_construct(self)
            logger = log4matlab('temp.txt');            
            delete(logger);
            
            logger = log4matlab('temp.txt',false);
            self.assertEquals(logger.append == false,1);
            delete(logger);
            
            logger = log4matlab('temp.txt',true);
            self.assertEquals(logger.append == true,1);
            delete(logger);
        end
        
%% test_mlog
        function test_mlog(self) %#ok<MANU>
            logger = log4matlab('temp.txt');     
            logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
            logger.mlog = {logger.WARN,'testing','Testing writing log file warning message'};
            logger.mlog = {logger.ERROR,'testing','Testing writing log file error message'};
            delete(logger);
        end               

%% test_SetLoggerLevel
        function test_SetLoggerLevel(self) %#ok<MANU>
            logger = log4matlab('temp.txt');     
            logger.SetLoggerLevel('testing',logger.WARN);
            logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
            logger.mlog = {logger.WARN,'testing','Testing writing log file warning message'};
            logger.mlog = {logger.ERROR,'testing','Testing writing log file error message'};
            delete(logger);
            
            logger = log4matlab('temp.txt');     
            logger.SetLoggerLevel('testing',logger.ERROR);
            logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
            logger.mlog = {logger.WARN,'testing','Testing writing log file warning message'};
            logger.mlog = {logger.ERROR,'testing','Testing writing log file error message'};
            delete(logger);            
        end     

%% test_GetLoggerLevel
        function test_GetLoggerLevel(self)
            logger = log4matlab('temp.txt');     
            logger.SetLoggerLevel('testing',logger.WARN);
            self.assertEquals(logger.WARN == logger.GetLoggerLevel('testing'),1);
            delete(logger);            
        end             
        
%% test_heavyUsage
        function test_heavyUsage(self)
            filename = 'temp.txt';
            try delete(filename); end %#ok<TRYNC>
            logger = log4matlab(filename);                 
            for i=1:1000
                logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
                logger.mlog = {logger.WARN,'testing2','Testing writing log file debug message'};
                logger.mlog = {logger.ERROR,'testing3','Testing writing log file debug message'};
            end
            fid = fopen(filename);
            data = textscan(fid, '%s','delimiter','\n');
            fclose(fid);
            self.assertEquals(size(data{1},1)==3000,1);
            delete(logger);            
        end        
        
%% test_manyCallsFewWrites
        function test_manyCallsFewWrites(self)        
            filename = 'temp.txt';
            try delete(filename); end %#ok<TRYNC>
            logger = log4matlab(filename);
            logger.mlog = {logger.DEBUG,'testing','Make sure there is at least one line'};
            logger.SetLoggerLevel('testing',logger.ERROR);
            logger.SetLoggerLevel('testing2',logger.ERROR);
            logger.SetLoggerLevel('testing3',logger.ERROR);
            for i=1:1000
                logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
                logger.mlog = {logger.WARN,'testing2','Testing writing log file debug message'};
                logger.mlog = {logger.WARN,'testing3','Testing writing log file debug message'};
            end
            fid = fopen(filename);
            data = textscan(fid, '%s','delimiter','\n');
            fclose(fid);
            self.assertEquals(size(data{1},1)==1,1);
            delete(logger);                    
        end
        
%% test_logFromNamespaceClass
        function test_logFromClass(self)
            filename = 'temp.txt';
            namespaceNClass = ['nameSpace.',mfilename('class')];
            logger = log4matlab(filename);
            logger.mlog = {logger.DEBUG,namespaceNClass,'Testing writing a class with a . in it'};
            fid = fopen(filename);
            data = textscan(fid, '%s','delimiter','\n');
            fclose(fid);
            self.assertEquals(size(data{1},1)==1,1);
            
            logger.SetLoggerLevel(namespaceNClass,logger.ERROR);
            self.assertEquals(logger.GetLoggerLevel(namespaceNClass)==logger.ERROR,1);
            for i=1:1000
                logger.mlog = {logger.DEBUG,namespaceNClass,'Testing writing log file debug message'};
                logger.mlog = {logger.WARN,namespaceNClass,'Testing writing log file warning message'};
                logger.mlog = {logger.ERROR,namespaceNClass,'Testing writing log file error message'};
            end
            fid = fopen(filename);
            data = textscan(fid, '%s','delimiter','\n');
            fclose(fid);
            % There 1000 error messages and 1 debug message from before
            self.assertEquals(size(data{1},1001)==1,1);
            
            delete(logger);                    
        end    
        
%% test_SetCommandWindowLevel       
        function test_SetCommandWindowLevel(self)
            logger = log4matlab('temp.txt');            
            
            % default is to only output ERRORs
            self.assertEquals(logger.commandWindowLevel == logger.NONE,1);
            display('You should see NO messages now:');
            logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
            logger.mlog = {logger.WARN,'testing','Testing writing log file warning message'};
            logger.mlog = {logger.ERROR,'testing','Testing writing log file error message'};                      
            
            logger.SetCommandWindowLevel(logger.ERROR);
            self.assertEquals(logger.commandWindowLevel == logger.ERROR,1);   
            display('You should see ONE messages now: an ERROR');
            logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
            logger.mlog = {logger.WARN,'testing','Testing writing log file warning message'};
            logger.mlog = {logger.ERROR,'testing','Testing writing log file error message'};                      
            
            logger.SetCommandWindowLevel(logger.WARN);
            self.assertEquals(logger.commandWindowLevel == logger.WARN,1);            
            display('You should see TWO messages now: a WARN and an ERROR');            
            logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
            logger.mlog = {logger.WARN,'testing','Testing writing log file warning message'};
            logger.mlog = {logger.ERROR,'testing','Testing writing log file error message'};                      
            
            logger.SetCommandWindowLevel(logger.DEBUG);
            self.assertEquals(logger.commandWindowLevel == logger.DEBUG,1);            
            display('You should see THREE messages now: a DEBUG, a WARN and an ERROR');
            logger.mlog = {logger.DEBUG,'testing','Testing writing log file debug message'};
            logger.mlog = {logger.WARN,'testing','Testing writing log file warning message'};
            logger.mlog = {logger.ERROR,'testing','Testing writing log file error message'};                      
            
            delete(logger);
        end                
        
%% test_logMatrixToString
        function test_logMatrixToString(self)
            filename = 'temp.txt';
            logger = log4matlab(filename);
            logger.mlog = {logger.DEBUG,'myClassName',['The transform is ',logger.MatrixToString(eye(4))]};
            fid = fopen(filename);
            data = textscan(fid, '%s','delimiter','\n');
            fclose(fid);
            self.assertEquals(size(data{1},1)==6,1);
            delete(logger);                    
        end  
        
%% test_logExceptionToString
        function test_logExceptionToString(self)
            filename = 'temp.txt';
            logger = log4matlab(filename);
            
            % An intentional error for testing purposes
            try a(0)
            catch ME
                logger.mlog = {logger.DEBUG,'myClassName',['There was an error',logger.ExceptionToString(ME)]};
            end                        
            fid = fopen(filename);
            data = textscan(fid, '%s','delimiter','\n');
            fclose(fid);
            self.assertEquals(size(data{1},1)==10,1);
            delete(logger);                    
        end
               
    end
end