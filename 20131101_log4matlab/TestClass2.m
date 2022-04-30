classdef TestClass2 < handle
    properties
        L = log4matlab('logFileName.log');
    end
    methods
        function Test(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),self.L.Me};
            a = 1;
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'a =',num2str(a)]};
        end
    end    
end

