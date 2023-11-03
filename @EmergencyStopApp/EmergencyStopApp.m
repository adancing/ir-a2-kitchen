classdef EmergencyStopApp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure matlab.ui.Figure
        StopButton matlab.ui.control.Button
        ResumeButton matlab.ui.control.Button
        ResetButton matlab.ui.control.Button
        RobotNameLabel matlab.ui.control.Label
    end
    
    properties (Access = private)
        RobotObj Robot
        RobotName string = "Robot"; % Default name
    end
    
    methods (Access = private)
        
    function createComponents(app)
        % Create UIFigure
        app.UIFigure = uifigure('Visible', 'off', ...
            'Position', [250, 700, 350, 250]);

        % Calculate positions based on the size of the UIFigure
        figureWidth = app.UIFigure.Position(3);
        figureHeight = app.UIFigure.Position(4);
    
        labelWidth = 200; % Width of the label
        labelX = (figureWidth - labelWidth) / 2; % Center X-position for the label
        labelY = 0.9 * figureHeight - 25; % Y-position for the label, moved down by 25 pixels

        buttonWidth = 150; % Width of the buttons
        buttonX = (figureWidth - buttonWidth) / 2; % Center X-position for the buttons
        buttonY = 0.7 * figureHeight - 50; % Y-position for the buttons, moved down by 25 pixels
        buttonSpacing = 30; % Vertical spacing between buttons

        % Create a label for the robot's name
        app.RobotNameLabel = uilabel(app.UIFigure);
        app.RobotNameLabel.InnerPosition = [labelX, labelY, labelWidth, 35];
        app.RobotNameLabel.Text = app.RobotName;
        app.RobotNameLabel.HorizontalAlignment = 'center';
        app.RobotNameLabel.FontSize = 20;
        app.RobotNameLabel.FontWeight = 'bold';
        app.RobotNameLabel.FontName = 'Comic Sans MS';

        % Create buttons
        app.StopButton = uibutton(app.UIFigure, 'push', ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'FontSize', 20, ...
            'BackgroundColor', [0.9350 0 0], ...
            'FontColor', 'w', ...
            'Text', ['EMERGENCY', newline, 'STOP'], ...
            'InnerPosition', [buttonX, buttonY, buttonWidth, 22+22+11], ...
            'ButtonPushedFcn', @(btn, event) stopRobot(app));

        app.ResumeButton = uibutton(app.UIFigure, 'push', ...
            'Text', '➤ Resume ➤', ...
            'InnerPosition', [buttonX, buttonY - buttonSpacing, buttonWidth, 22], ...
            'ButtonPushedFcn', @(btn, event) resumeRobot(app), ...
            'BackgroundColor', [0.5961 1 0.5961]);

        app.ResetButton = uibutton(app.UIFigure, 'push', ...
            'Text', '↻ Reset ↻', ...
            'InnerPosition', [buttonX, buttonY - 2*buttonSpacing, buttonWidth, 22], ...
            'ButtonPushedFcn', @(btn, event) resetRobot(app), ...
            'BackgroundColor', [0.9882 0.9569 0.6392]);

        app.UIFigure.Visible = 'on';
    end

        function stopRobot(app)
            app.RobotObj.eStop();
        end
        
        function resumeRobot(app)
            app.RobotObj.resumeOperations();
        end
        
        function resetRobot(app)
            app.RobotObj.resetEStop();
        end
    end
    
    methods (Access = public)
        
        function app = EmergencyStopApp(robotObj, robotName)
            % Store robot object and name
            app.RobotObj = robotObj;
            app.RobotName = robotName;
            
            % Create and configure components
            createComponents(app);
        end
    end
end
