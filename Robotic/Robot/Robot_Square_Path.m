L1 = 0.2;  
L2 = 0.15; 

robot = rigidBodyTree('DataFormat','column');

body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1, [L1 0 0 0], 'dh');
body1.Joint = jnt1;
addBody(robot, body1, 'base');

body2 = rigidBody('link2');
jnt2 = rigidBodyJoint('jnt2','revolute');
setFixedTransform(jnt2, [L2 0 0 0], 'dh'); 
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

points = [0.15, 0.01;  
          0.25, 0.01;  
          0.25, 0.11;  
          0.15, 0.11;   
          0.15, 0.01]; 

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0 0 0 1 1 0]; 
initial_guess = robot.homeConfiguration;

fprintf('Calculating Joint Angles for Square Path:\n');
for i = 1:size(points, 1)
    target_pos = [points(i,1) points(i,2) 0]; 
    tform = trvec2tform(target_pos);
    
    [configSol, solInfo] = ik('link2', tform, weights, initial_guess);
    
    fprintf('Point %d (%0.2f, %0.2f) -> Theta1: %0.2f, Theta2: %0.2f\n', ...
        i, points(i,1), points(i,2), rad2deg(configSol(1)), rad2deg(configSol(2)));
    
    initial_guess = configSol; 
end

show(robot, configSol);
title('Robot at Final Square Corner');
axis([-0.4 0.4 -0.4 0.4]);