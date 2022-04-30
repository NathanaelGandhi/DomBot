%% Example 1 - Writing creating and writing log
L = log4matlab('logFileName1.log');

ScriptName = 'Example';

L.mlog = {L.DEBUG,ScriptName,'This is a debug message'}; 
L.mlog = {L.WARN,ScriptName,'This is a warning message'}; 
L.mlog = {L.ERROR,ScriptName,'This is a error message'}; 

L.SetLoggerLevel(ScriptName,L.WARN) 
L.mlog = {L.DEBUG,ScriptName,'This is a debug message, after setting to WARNs only'}; 
L.mlog = {L.WARN,ScriptName,'This is a warning message, after setting to WARNs only'}; 
L.mlog = {L.ERROR,ScriptName,'This is a error message, after setting to WARNs only'}; 
                    
L.SetLoggerLevel(ScriptName,L.ERROR) 
L.mlog = {L.DEBUG,ScriptName,'This is a debug message, after setting to ERRORs only'}; 
L.mlog = {L.WARN,ScriptName,'This is a warning message, after setting to ERRORs only'}; 
L.mlog = {L.ERROR,ScriptName,'This is a error message, after setting to ERRORs only'}; 

switch L.getLoggerLevel('myClassName') 
    case L.DEBUG
        disp('LoggerLevel currently set to DEBUG');
    case L.WARN
        disp('LoggerLevel currently set to WARN');
    case L.ERROR
        disp('LoggerLevel currently set to ERROR');
end

edit('logFileName1.log');

uiwait(msgbox('Press OK to continue writing outputs of the next two classes to the second log file'));

%% Example 2 - How to use in a large project with many classes

% Create a normal class with singleton log4matlab handle L which is the logger
t1 = TestClass1;
t1.Run;

% Create a class in the namespace which gets the same singleton log4matlab handle, L
t2 = Namespace.TestClass1;
t2.Run;

% Have a look at the log file (should use a better text editor than matlab)
edit('logFileName2.log');

