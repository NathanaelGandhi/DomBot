classdef TestClass1 < handle
    %TESTCLASS1 This is for testing the log4matlab class
    
    properties(Constant)
        L = SingleInstance.Logger;
    end
    
    methods
        function self = TestClass1()
        end
        
        function Run(self)
            a = rand;
            for i=1:100
                a = a+i;
                str = num2str(sqrt(a));
                
                self.L.mlog = {self.L.DEBUG ...
                              ,mfilename('class') ...
                              ,['The value of a = ', num2str(a),' and the sqrt of this is ',str]};
                          
                if mod(i,3)==0
                    self.L.mlog = {self.L.WARN ...
                                  ,mfilename('class') ...
                                  ,['This a test warning message for when mod(i,3)==0. Currently i=',num2str(i)]};
                end
                
                if mod(i,10)==0
                    self.L.mlog = {self.L.ERROR ...
                                  ,mfilename('class') ...
                                  ,['This a test error message for when mod(i,10)==0. Currently i=',num2str(i)]};
                end
            end
        end
    end
    
end

