function [object] = BuildEnvironment_UTS(Robot, objData)
% BuildEnvironment loads, positions, and orients all the objects in the environment

hold on;

% Load Objects (All objects loaded in BuildEnvironment_Group)

% Initialize bricks cell array based on the number of bricks
object = cell(size(objData, 1), 1);

% Load and orient bricks based on the provided data
for i = 1:size(objData, 1)
   position = objData(i, 1:3);
   orientation = objData(i, 4:6); % Assuming orientation is given in Euler angles [rotX, rotY, rotZ]
   object{i} = LoadObject("plate.ply", position, orientation); 
end

% Set robot base positions
% Translation of the robot base
T_trans = transl(0, 2, 0.525);

% Rotation of the robot base about the Z-axis by 45 degrees
T_rot = trotz(pi/2);

% Combine translation and rotation
T_base_new = T_trans * T_rot;

% Apply the new base transformation to the robot
Robot.model.base = T_base_new;


end
