clf

cobotLog = Log;
tempPose = transl(0,0,0);
dombot = MyCobot(cobotLog, 2, tempPose);

%% CalculateTraj and RunTraj Tests
steps = 250;
% nextPose = transl(0.2, 0.1, 0.1);
nextPose2 = transl(0.073, 0.066, 0.294);   % Default spawn pose
nextPose1 = transl(0.28, 0.066, 0.175);  % MAX X
% nextPose2 = transl(0.066, 0.28, 0.175); % MAX Y

while(1)
    % Calculates the trajectory (using RMRC)
    dombot.CalculateTraj(nextPose1, steps);
    for i=1:steps
        % Runs trajectory that was just calculated
        dombot.RunTraj();
    end

    dombot.CalculateTraj(nextPose2, steps);
    for i=1:steps
        dombot.RunTraj();
    end
end

%% Notes
%         % You can even break out of the for loop and the DomBot will just stop at  
%         % that position until its next trajectory is calculated
%         if (i > 243)
%             break;
%         end