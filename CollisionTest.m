cobotLog = Log;
tempPose = transl(0,0,0);
dombot = MyCobot(cobotLog, 2, tempPose);

signLog = Log;
tempPose = transl(1,1,0);
sign = StopSign(signLog, 3, tempPose);

faceA = sign.model.Faces;

% sign.UpdatePose(transl(0,1,0));
% 
% faceB = sign.model.Faces;

% [tr, allTR] = dombot.model.fkine(dombot.model.getpos);

% for i = 1:dombot.model.n
%     