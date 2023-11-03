function ObjectPlacementGROUP_ROBOT(control_GROUP, Robo_Models, bricks, XYZ_OBJECT, FINAL_POSE_OBJECT, gr, gr1, gr2, gr3, emergencyStopRobot, masterButton)
    % This function manages the robotic operation for brick placement.

    % Count the total number of bricks based on the input matrix dimensions
    numberOfBricks = size(XYZ_OBJECT, 1);

    % Define the maximum reach distance of the UR3 robot
    MaxReachDistance = 2;
    
    % Define the lift distance for the end-effector before and after pick/place
    liftDistance = 0.15; % 10 cm lift

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
            disp("ROBOT PROCEED: Object within reach for robot");

            brickIdx = reachDistances(idx, 1);
            reachDistances(idx, :) = [];

            % Set the pick position for the brick
            PickPosition = XYZ_OBJECT(brickIdx, 1:3);
            PickOrientation = XYZ_OBJECT(brickIdx, 4:6);
            PlacePosition = FINAL_POSE_OBJECT(1, 1:3);
            PlaceOrientation = FINAL_POSE_OBJECT(1, 4:6);
            FINAL_POSE_OBJECT(1, :) = [];
            Brick = bricks{brickIdx};
        else
            % If no bricks are within the robot's reach
            disp("ROBOT STANDBY: No bricks within reach for robot");

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
        control_GROUP.MoveEndEffectorToPoint(Robo_Models, 0, PickPositionUp, gr, gr1, gr2, gr3, 0, emergencyStopRobot, masterButton);

        % Start the operation to pick up the brickz`
        disp("Picking up object");
        control_GROUP.MoveEndEffectorToPoint(Robo_Models, 0, [PickPosition, PickOrientation], gr, gr1, gr2, gr3, 0, emergencyStopRobot, masterButton);

        % Move back to the adjusted pick position with the brick
        disp("Lifting the object");
        control_GROUP.MoveEndEffectorToPoint(Robo_Models, Brick, PickPositionUp, gr, gr1, gr2, gr3, 1, emergencyStopRobot, masterButton);

        % Adjust place position to go upwards before placing
        PlacePositionUp = [PlacePosition + [0, 0, liftDistance], PlaceOrientation];

        % Move to the adjusted place position
        disp("Moving to adjusted place position");
        control_GROUP.MoveEndEffectorToPoint(Robo_Models, Brick, PlacePositionUp, gr, gr1, gr2, gr3, 1, emergencyStopRobot, masterButton);
        

        % Start the operation to place the brick
        disp("Placing object");
        control_GROUP.MoveEndEffectorToPoint(Robo_Models, Brick, [PlacePosition, PlaceOrientation], gr, gr1, gr2, gr3, 1, emergencyStopRobot, masterButton);
        

        % Move back to the adjusted place position without the brick
        disp("Retreating from place position");
        control_GROUP.MoveEndEffectorToPoint(Robo_Models, 0, PlacePositionUp, gr, gr1, gr2, gr3, 0, emergencyStopRobot, masterButton);
    end

    % Return the robot to its initial position after completing the operations
    currentJointPosition = Robo_Models.model.getpos();
    targetJointPosition = [0, 0, 0, 0, 0, 0];
    trajectoryLength = 15;
    jointTrajectory = jtraj(currentJointPosition, targetJointPosition, trajectoryLength);

    % Animate the robot's movement back to its initial position
    for trajStep = 1:size(jointTrajectory, 1)
        Q = jointTrajectory(trajStep, :);
        Robo_Models.model.animate(Q);
        
        Fkine = Robo_Models.model.fkine(Q);
        fk = [Fkine.n Fkine.o Fkine.a Fkine.t; 0 0 0 1];
            gr.base = fk * transl(-0.02, 0, 0);
            gr.animate([0, deg2rad(145), deg2rad(-45), 0]);
            gr1.base = fk * transl(0.02, 0, 0);
            gr1.animate([pi, deg2rad(145), deg2rad(-45), 0]);
            gr2.base = fk * transl(0, -0.02, 0) * trotz(deg2rad(90));
            gr2.animate([0, deg2rad(145), deg2rad(-45), 0]);
            gr3.base = fk * transl(0, 0.02, 0) * trotz(deg2rad(-90));
            gr3.animate([0, deg2rad(145), deg2rad(-45), 0]);
        drawnow();
    end

    % Display a message indicating the completion of the brick wall assembly
    disp("Brick wall assembly complete");
end

