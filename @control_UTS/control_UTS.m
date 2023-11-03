classdef control_UTS < handle
    % CONTROL contains functions and parameters to control robot movements.
    
    methods
        function MoveEndEffectorToPoint(~, Robot, Object, RobotEndPoint, emergencyButton, masterButton)
            fprintf('Starting the next movement... \n');
            
            CurrentTrajStep = 1;

            % Define the trajectory length
            trajectoryLength = 50;

            % If the operation is not paused or reset, calculate the trajectory
                % Get the current forward kinematics of the UR3 robot
                StartFkine = Robot.model.fkine(Robot.model.getpos());
                fk = StartFkine.t;

                % Calculate the change in position from the current to the desired endpoint
                DeltaPosition = [(RobotEndPoint(1) - fk(1)), (RobotEndPoint(2) - fk(2)), (RobotEndPoint(3) - fk(3) + 0.095)];

                % Calculate the end position forward/inverse kinematics
                EndFkine = transl(DeltaPosition) * [StartFkine.n StartFkine.o StartFkine.a StartFkine.t; 0 0 0 1];
                EndIkcon = Robot.model.ikcon(EndFkine);
                disp('Inverse Kinematics Solution:');
                disp(EndIkcon);

                % Determine the joint positions required for the movement using a quintic trajectory
                JointTrajectory = jtraj(Robot.model.getpos(), EndIkcon, trajectoryLength);

            % Loop over each step in the joint trajectory
            while CurrentTrajStep <= size(JointTrajectory, 1)

            % Check for emergency stop
            if strcmp(emergencyButton.State, 'Stopped')
                pause(0.1);
                continue;  % Skip to the next iteration of the while loop
            end

                        % Check for emergency stop
            if strcmp(emergencyButton.State, 'Paused')
                pause(0.1);
                continue;  % Skip to the next iteration of the while loop
            end

                             % Check for emergency stop
            if strcmp(masterButton.State, 'Stopped')
                pause(0.1);
                continue;  % Skip to the next iteration of the while loop
            end

                        % Check for emergency stop
            if strcmp(masterButton.State, 'Paused')
                pause(0.1);
                continue;  % Skip to the next iteration of the while loop
            end

                Q = JointTrajectory(CurrentTrajStep,:);
                Robot.model.animate(Q);

                Fkine = Robot.model.fkine(Q);

                % If a brick is being held, move it with the robot's end effector
                if Object ~= 0
                    fxyz = Fkine.t;
                    MoveObject(Object, [fxyz(1), fxyz(2), fxyz(3) - 0.095], 0);
                end
                drawnow();

                CurrentTrajStep = CurrentTrajStep + 1;  % Update the current trajectory step
            end

            fprintf('Movement complete.\n');
        end


    end
end