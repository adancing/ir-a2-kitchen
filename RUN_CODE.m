%% Start Profiling for Computational Efficiency
% This will keep track of the code's performance metrics
profile on;

%% Clear workspace and figures
% Removes all variables from the workspace and clears any plotted figures
clear all;
clf;

%% Initialize Robot Models
% Create an instance of the IRB120 robot model
GroupROBOT = GroupROBOT();
utsROBOT = UR3(); % Create an instance of the UR3 robot model

Robot_Models = {GroupROBOT, utsROBOT};
% Create an instance of the Robot class with emergency stop functionality
Group_ROBOT = Robot();
UTS_ROBOT = Robot();
Master_Button = Robot();

app1 = EmergencyStopApp(Group_ROBOT, 'groupROBOT');
app2 = EmergencyStopApp(UTS_ROBOT, 'utsROBOT');
app3 = EmergencyStopApp(Master_Button, 'masterBUTTON');

%% Define Starting positions of the bricks with orientation (position: x, y, z; orientation: rotX, rotY, rotZ)
% Matrix stores the initial position and orientation of each brick
INITIAL_Position_Of_Object_GroupROBOT = [
    0.3, 0.2, 0.525, 0, 0, 0;
    0.3, 0.325, 0.525, 0, 0, 0;
    0.425, 0.2, 0.525, 0, 0, 0;
    0.425, 0.325, 0.525, 0, 0, 0;
    0.55, 0.2, 0.525, 0, 0, 0;
];

INITIAL_Position_Of_Object_utsROBOT = [
    -0.22, 2.1, 0.525 + 0.010*2, 0, 0, 0;
    -0.22, 2.1, 0.525 + 0.010, 0, 0, 0;
    -0.22, 2.1, 0.525, 0, 0, 0;
];

%% Build Environment using position and orientation data
% Use the provided positions and orientations to create and visualize the bricks in the environment
objectGROUP = BuildEnvironment_GROUP(GroupROBOT, INITIAL_Position_Of_Object_GroupROBOT);
objectUTS = BuildEnvironment_UTS(utsROBOT, INITIAL_Position_Of_Object_utsROBOT);

axis([-4 2 -1.5 4.5 -0.05 2.5]);          % Full environment view
% axis([-1.2 1.2 -0.7 2.65 -0.05 1.6]);   % Table view

%% Define and Plot Grippers
% Define the structure and joint properties of the grippers
L_groupROBOT(1) = Revolute('d', 0.01, 'a', 0, 'alpha', pi/2);
L_groupROBOT(2) = Revolute('d', 0, 'a', 0.04, 'alpha', 0);
L_groupROBOT(3) = Revolute('d', 0, 'a', 0.04, 'alpha', 0);  % Define L3
L_groupROBOT(4) = Revolute('d', 0, 'a', 0.03, 'alpha', 0);  % Define L4

% Create three instances of grippers with the above structure
gr_groupROBOT = SerialLink(L_groupROBOT, 'name', 'gr');
gr1_groupROBOT = SerialLink(L_groupROBOT, 'name', 'gr1');
gr2_groupROBOT = SerialLink(L_groupROBOT, 'name', 'gr2');
gr3_groupROBOT = SerialLink(L_groupROBOT, 'name', 'gr3');

% Plot the initial position of the grippers
gr_groupROBOT.plot([0, deg2rad(150), deg2rad(180), 0]);
hold on;
gr1_groupROBOT.plot([0, deg2rad(150), deg2rad(180), 0]);
hold on;
gr2_groupROBOT.plot([0, deg2rad(150), deg2rad(180), 0]);
hold on;
gr3_groupROBOT.plot([0, deg2rad(150), deg2rad(180), 0]);
hold on;

%% Animate Robot Initial Position
% Display the robot's initial position in the environment
Robot_Models{1}.model.animate(Robot_Models{1}.model.getpos());
drawnow();
% Display the robot's initial position in the environment
Robot_Models{2}.model.animate(Robot_Models{2}.model.getpos());
drawnow();

%% Define Object Placement Parameters
% Matrix to store the desired FRUIT positions to build the wall
objectPosition_groupROBOT = [
    0.200, -0.200, 0.525 + 0.008, 0, 0, 0;
    0.200, -0.300, 0.525 + 0.008, 0, 0, 0;
    0.300, -0.200, 0.525 + 0.008, 0, 0, 0;
    0.300, -0.300, 0.525 + 0.008, 0, 0, 0;
    0.250, -0.250, 0.525 + 0.076, 0, 0, 0;
];

% Matrix to store the desired PLATE positions to build the wall
objectPosition_utsROBOT = [
    0, 2.3, 0.525, 0, 0, 0;
    0.5, 2, 0.525, 0, 0, 0;
    -0.5,2, 0.525, 0, 0, 0;
];

%% Update Pick and Place Positions
% For picking, we are considering just positions and a default orientation
objectPICKposition_groupROBOT = INITIAL_Position_Of_Object_GroupROBOT;
objectPICKposition_groupROBOT(:, 1:3) = objectPICKposition_groupROBOT(:, 1:3) + [0, 0, 0];

% For picking, we are considering just positions and a default orientation
objectPICKposition_utsROBOT = INITIAL_Position_Of_Object_utsROBOT;
objectPICKposition_utsROBOT(:, 1:3) = objectPICKposition_utsROBOT(:, 1:3) + [0, 0, 0];

% For placing, we are considering positions and the given orientations
objectPLACEpositionGROUP_ROBOT = objectPosition_groupROBOT;
objectPLACEpositionGROUP_ROBOT(:, 1:3) = objectPLACEpositionGROUP_ROBOT(:, 1:3) + [0, 0, 0];

% For placing, we are considering positions and the given orientations
objectPLACEpositionUTS_ROBOT = objectPosition_utsROBOT;
objectPLACEpositionUTS_ROBOT(:, 1:3) = objectPLACEpositionUTS_ROBOT(:, 1:3) + [0, 0, 0];

%% Start Brick Placement Sequence
ObjectPlacementGROUP_ROBOT(control_GROUP, Robot_Models{1}, objectGROUP, objectPICKposition_groupROBOT, objectPLACEpositionGROUP_ROBOT, gr_groupROBOT, gr1_groupROBOT, gr2_groupROBOT, gr3_groupROBOT,Group_ROBOT, Master_Button);

ObjectPlacementUTS_ROBOT(control_UTS, Robot_Models{2}, objectUTS, objectPICKposition_utsROBOT, objectPLACEpositionUTS_ROBOT, UTS_ROBOT,Master_Button)