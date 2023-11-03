classdef control_GROUP < handle
    % CONTROL contains functions and parameters to control robot movements.
    
    methods
        function MoveEndEffectorToPoint(~, Robot, Object, RobotEndPoint, gr, gr1, gr2, gr3, a, emergencyButton, masterButton)
            fprintf('Starting the next movement... \n');
            
            CurrentTrajStep = 1;

            % Define the trajectory length
            trajectoryLength = 20;

            % If the operation is not paused or reset, calculate the trajectory
                % Get the current forward kinematics of the UR3 robot
                StartFkine = Robot.model.fkine(Robot.model.getpos());
                fk = StartFkine.t;

                % Calculate the change in position from the current to the desired endpoint
                DeltaPosition = [(RobotEndPoint(1) - fk(1) - 0.009), (RobotEndPoint(2) - fk(2) - 0.009), (RobotEndPoint(3) - fk(3) + 0.11)];

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
                Fkine1 = [Fkine.n Fkine.o Fkine.a Fkine.t; 0 0 0 1];

                % Update grippers' position based on 'a' parameter value
                if a == 0
                    % Open position
                    gr.base = Fkine1 * transl(-0.02, 0, 0);
                    gr.animate([0, deg2rad(145), deg2rad(-45), 0]);
                    gr1.base = Fkine1 * transl(0.02, 0, 0);
                    gr1.animate([pi, deg2rad(145), deg2rad(-45), 0]);
                    gr2.base = Fkine1 * transl(0, -0.02, 0) * trotz(deg2rad(90));
                    gr2.animate([0, deg2rad(145), deg2rad(-45), 0]);
                    gr3.base = Fkine1 * transl(0, 0.02, 0) * trotz(deg2rad(-90));
                    gr3.animate([0, deg2rad(145), deg2rad(-45), 0]);
                elseif a == 1
                    % Close position
                    gr.base = Fkine1 * transl(-0.02, 0, 0);
                    gr.animate([0, deg2rad(140), deg2rad(-45), deg2rad(-20)]);
                    gr1.base = Fkine1 * transl(0.02, 0, 0);
                    gr1.animate([pi, deg2rad(140), deg2rad(-45), deg2rad(-20)]);
                    gr2.base = Fkine1 * transl(0, -0.02, 0) * trotz(deg2rad(90));
                    gr2.animate([0, deg2rad(140), deg2rad(-45), deg2rad(-20)]);
                    gr3.base = Fkine1 * transl(0, 0.02, 0) * trotz(deg2rad(-90));
                    gr3.animate([0, deg2rad(140), deg2rad(-45), deg2rad(-20)]);
                end
                drawnow();

                % If a brick is being held, move it with the robot's end effector
                if Object ~= 0
                    fxyz = Fkine.t;
                    MoveObject(Object, [fxyz(1), fxyz(2), fxyz(3) - 0.055], 0);
                end
                drawnow();

                CurrentTrajStep = CurrentTrajStep + 1;  % Update the current trajectory step
            end

            fprintf('Movement complete.\n');
        end


    end
end