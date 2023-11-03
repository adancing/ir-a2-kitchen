function GROUPGUI()
    % Create a figure for the GUI
    fig = figure('Name', 'IRB120 Robot Control', 'NumberTitle', 'off', 'Position', [100, 100, 600, 1000]);

    % Create an axes for plotting the robot
    ax = axes('Position', [0.1, 0.6, 0.8, 0.3]);
    hold(ax, 'on');
    
    % Create robot model
    robot = GroupROBOT();

    % Set default joint values
    jointValues = zeros(1, 6);

    % Define and Plot Grippers
    L_groupROBOT(1) = Revolute('d', 0.01, 'a', 0, 'alpha', pi/2);
    L_groupROBOT(2) = Revolute('d', 0, 'a', 0.04, 'alpha', 0);
    L_groupROBOT(3) = Revolute('d', 0, 'a', 0.04, 'alpha', 0);
    L_groupROBOT(4) = Revolute('d', 0, 'a', 0.03, 'alpha', 0);

    gr = SerialLink(L_groupROBOT, 'name', 'gr');
    gr1 = SerialLink(L_groupROBOT, 'name', 'gr1');
    gr2 = SerialLink(L_groupROBOT, 'name', 'gr2');
    gr3 = SerialLink(L_groupROBOT, 'name', 'gr3');

    gr.plot([0, deg2rad(145), deg2rad(-45), 0]);
    gr1.plot([pi, deg2rad(145), deg2rad(-45), 0]);
    gr2.plot([0, deg2rad(145), deg2rad(-45), 0]);
    gr3.plot([0, deg2rad(145), deg2rad(-45), 0]);

% Label for joint control sliders
uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
          'Position', [0.1, 0.4, 0.8, 0.02], 'String', 'Joint Control', ...
          'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Add sliders for joint control
for i = 1:6
    uicontrol('Parent', fig, 'Style', 'slider', 'Min', -pi, 'Max', pi, 'Value', jointValues(i), ...
              'Units', 'normalized', 'Position', [0.1, 0.4 - 0.02*i, 0.8, 0.02], ...
              'Tag', num2str(i), 'Callback', @joint_slider_callback);
    uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
              'Position', [0.05, 0.4 - 0.02*i, 0.05, 0.02], 'String', ['J' num2str(i)]);
end

% Label for Cartesian control sliders
uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
          'Position', [0.1, 0.2, 0.8, 0.02], 'String', 'Cartesian Control', ...
          'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Add sliders for Cartesian control
labels = {'X', 'Y', 'Z'};
for i = 1:3
    uicontrol('Parent', fig, 'Style', 'slider', 'Min', -1, 'Max', 1, 'Value', 0, ...
              'Units', 'normalized', 'Position', [0.1, 0.2 - 0.02*i, 0.8, 0.02], ...
              'Tag', labels{i}, 'Callback', @cartesian_slider_callback);
    uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
              'Position', [0.05, 0.2 - 0.02*i, 0.05, 0.02], 'String', labels{i});
end


    function joint_slider_callback(src, ~)
        % Get joint number from slider tag
        jointNum = str2double(src.Tag);
        % Update joint value    
        jointValues(jointNum) = src.Value;
        % Update robot plot
        jointValues = update_plot(jointValues);
    end

function cartesian_slider_callback(src, ~)
    % Get the current end effector pose
    T_end_effector = robot.model.fkine(jointValues);
    
    % Get axis of translation from slider tag
    ax = src.Tag;
    
    % Update pose based on slider value
    if ax == 'X'
        deltaT = SE3(src.Value, 0, 0); % Create a translation SE3 object
    elseif ax == 'Y'
        deltaT = SE3(0, src.Value, 0);
    elseif ax == 'Z'
        deltaT = SE3(0, 0, src.Value);
    end
    
    % Apply the translation to the end effector pose
    T_end_effector_new = T_end_effector * deltaT;
    
    % Perform inverse kinematics to find the new joint values
    jointValues = robot.model.ikcon(T_end_effector_new, jointValues);
    
    % Update robot plot
    jointValues = update_plot(jointValues);
    
    % Reset slider value to zero
    src.Value = 0;
end

    function jointValues = update_plot(jointValues)
        robot.model.animate(jointValues);
        update_gripper(jointValues);
    end
function update_gripper(jointValues)
    Fkine1 = robot.model.fkine(jointValues).T;

    gr.base = Fkine1 * transl(-0.02, 0, 0);
    gr.animate([0, deg2rad(145), deg2rad(-45), 0]);

    gr1.base = Fkine1 * transl(0.02, 0, 0);
    gr1.animate([pi, deg2rad(145), deg2rad(-45), 0]);

    gr2.base = Fkine1 * transl(0, -0.02, 0) * trotz(deg2rad(90));
    gr2.animate([0, deg2rad(145), deg2rad(-45), 0]);

    gr3.base = Fkine1 * transl(0, 0.02, 0) * trotz(deg2rad(-90));
    gr3.animate([0, deg2rad(145), deg2rad(-45), 0]);
end

end
