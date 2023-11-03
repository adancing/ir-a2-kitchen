function ObjectPlacementUTS_ROBOT(control_UTS, Robo_Models, bricks, XYZ_Position, Object_Final_Pose, emergencyStopRobot, masterButton)
     % This function manages the robotic operation for brick placement.

    % Count the total number of bricks based on the input matrix dimensions
    numberOfBricks = size(XYZ_Position, 1);

    % Define the maximum reach distance of the UR3 robot
    MaxReachDistance = 2;
    
    % Define the lift distance for the end-effector before and after pick/place
    liftDistance = 0.1; % 10 cm lift

    reachDistances = zeros(numberOfBricks, 3);
    BasePosition = Robo_Models.model.base.t';

    % Calculate the distances from the robot's base to each brick
    for i = 1:numberOfBricks
        midPoint = mean(bricks{i}.Vertices);
        reachDistances(i, :) = [i, norm(midPoint - BasePosition), 0];
    end

    % Start brick placement operations until all bricks have been processed
    while ~isempty(reachDistances)

        [ClosestBrickDistance, idx] = min(reachDistances(:, 2));

        % If the closest brick is within the robot's reach
        if ClosestBrickDistance < MaxReachDistance
            disp("UR3 PROCEED: Brick within reach for UR3");

            brickIdx = reachDistances(idx, 1);
            reachDistances(idx, :) = [];

            % Set the pick position for the brick
            PickPosition = XYZ_Position(brickIdx, 1:3);
            PickOrientation = XYZ_Position(brickIdx, 4:6);
            PlacePosition = Object_Final_Pose(1, 1:3);
            PlaceOrientation = Object_Final_Pose(1, 4:6);
            Object_Final_Pose(1, :) = [];
            Brick = bricks{brickIdx};
        else
            % If no bricks are within the robot's reach
            disp("UR3 STANDBY: No bricks within reach for UR3");

            % Set the pick and place positions to the robot's current position
            Fk = Robo_Models.model.fkine([0,0,0,0,0,0]);
            PickPosition = Fk.t';
            PickOrientation = tr2rpy(Fk);  % Extract orientation
            PlacePosition = Fk.t';
            PlaceOrientation = tr2rpy(Fk);

            Brick = 0;
        end

        % Adjust pick position to go upwards before picking
        PickPositionUp = [PickPosition + [0, 0, liftDistance], PickOrientation];
        
        % Move to the adjusted pick position
        disp("Moving to adjusted pick position");
        control_UTS.MoveEndEffectorToPoint(Robo_Models, 0, PickPositionUp, emergencyStopRobot, masterButton);

        % Start the operation to pick up the brick
        disp("Picking up bricks");
        control_UTS.MoveEndEffectorToPoint(Robo_Models, 0, [PickPosition, PickOrientation], emergencyStopRobot, masterButton);

        % Move back to the adjusted pick position with the brick
        disp("Lifting the brick");
        control_UTS.MoveEndEffectorToPoint(Robo_Models, Brick, PickPositionUp, emergencyStopRobot, masterButton);

        % Adjust place position to go upwards before placing
        PlacePositionUp = [PlacePosition + [0, 0, liftDistance], PlaceOrientation];

        % Move to the adjusted place position
        disp("Moving to adjusted place position");
        control_UTS.MoveEndEffectorToPoint(Robo_Models, Brick, PlacePositionUp, emergencyStopRobot, masterButton);
        

        % Start the operation to place the brick
        disp("Placing bricks");
        control_UTS.MoveEndEffectorToPoint(Robo_Models, Brick, [PlacePosition, PlaceOrientation], emergencyStopRobot, masterButton);
        

        % Move back to the adjusted place position without the brick
        disp("Retreating from place position");
        control_UTS.MoveEndEffectorToPoint(Robo_Models, 0, PlacePositionUp, emergencyStopRobot, masterButton);
    end

    % Return the robot to its initial position after completing the operations
    currentJointPosition = Robo_Models.model.getpos();
    targetJointPosition = [0, 0, 0, 0, 0, 0, 0];
    trajectoryLength = 5;
    jointTrajectory = jtraj(currentJointPosition, targetJointPosition, trajectoryLength);

    % Animate the robot's movement back to its initial position
    for trajStep = 1:size(jointTrajectory, 1)
        Q = jointTrajectory(trajStep, :);
        Robo_Models.model.animate(Q);
        
        drawnow();
    end

    % Display a message indicating the completion of the brick wall assembly
    disp("Brick wall assembly complete");
end
