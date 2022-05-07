clf

cobotLog = Log;
tempPose = transl(0,0,0);
dombot = MyCobot(cobotLog, 2, tempPose);

nextPose = transl(0.2, 0.1, 0.1);
% Calculates the trajectory (using RMRC)
dombot.CalculateTraj(nextPose, 500);

% Runs trajectory that was just calculated
for i=1:500
dombot.RunTraj(i);

% You can even break out of the for loop and the DomBot will just stop at  
% that position until its next trajectory is calculated

% if (i > 243)
%     break;
% end
end

nextPose = transl(0.2, 0.1, 0);
dombot.CalculateTraj(nextPose, 200);
for i=1:200
dombot.RunTraj(i);
end