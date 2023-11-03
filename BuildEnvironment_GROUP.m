function [object] = BuildEnvironment_GROUP(Robot, objData)
% BuildEnvironment loads, positions, and orients all the objects in the environment

hold on;

% Load objects
LoadObject("rectable.ply", [0,1,0], [0, 0, pi/2]);
LoadObject("wooden_fruit_bowl2.ply", [0.25,-0.25,0.525], [0, 0, 0]);
LoadObject("emergencyStopWallMounted.ply", [-0.65, 2.35,0.4], [-pi/2, 0, 0]);
LoadObject("emergencyStopWallMounted.ply", [0.54,-0.35,0.4], [-pi/2, 0, 0]);
LoadObject("fireExtinguisher.ply", [0.6,2.25,0], [0, 0, 0]);
LoadObject("chair.ply", [-1,0,0], [0, 0, pi/2]);
LoadObject("chair.ply", [-1,1,0], [0, 0, pi/2]);
LoadObject("chair.ply", [-1,2,0], [0, 0, pi/2]);
LoadObject("chair.ply", [0,2.8,0], [0, 0, 0]);
LoadObject("chair.ply", [1,0,0], [0, 0, -pi/2]);
LoadObject("chair.ply", [1,1,0], [0, 0, -pi/2]);
LoadObject("chair.ply", [1,2,0], [0, 0, -pi/2]);
LoadObject("RobotMovingWarning.ply", [-0.5,2.35,0.525], [0, 0, -pi/2]);
LoadObject("RobotMovingWarning.ply", [0.5,-0.30,0.525], [0, 0, pi/2]);
LoadObject("kitchencolouredv3.ply", [-2,-1,0], [0, 0, pi/2]);
LoadObject("redsiren.ply", [0,1,0.525], [0, 0, 0]);
LoadObject("barrier.ply", [-3,3,0], [0, 0, pi/2]); % next to kitchen along x axis
LoadObject("barrier.ply", [-2,4,0], [0, 0, 0]); 
LoadObject("barrier.ply", [0,4,0], [0, 0, 0]); 
LoadObject("barrier.ply", [1.5,3.2,0], [0, 0, pi/2]);
LoadObject("barrier.ply", [1.5,1.4,0], [0, 0, pi/2]);
LoadObject("barrier.ply", [1.5,-0.6,0], [0, 0, pi/2]);
LoadObject("barrier.ply", [0,-1.4,0], [0, 0, 0]); 
LoadObject("barrier.ply", [-2,-1.4,0], [0, 0, 0]);
% Initialize bricks cell array based on the number of bricks
object = cell(size(objData, 1), 1);

% Load and orient bricks based on the provided data
for i = 1:size(objData, 1)
   position = objData(i, 1:3);
   orientation = objData(i, 4:6); % Assuming orientation is given in Euler angles [rotX, rotY, rotZ]
   object{i} = LoadObject("orange2.ply", position, orientation); 
end

% Set robot base positions
Robot.model.base = transl(0, 0, 0.525);

end
