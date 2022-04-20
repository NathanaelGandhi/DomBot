clf
dombot = myCobot;


t1 = transl(-0.05, -0.27, 0.1);
pause(2);
dombot.calculateTraj(t1,100);
dombot.runTraj();

