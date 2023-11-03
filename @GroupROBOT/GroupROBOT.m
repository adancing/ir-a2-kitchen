classdef GroupROBOT < RobotBaseClass

    properties(Access = public)  
        plyFileNameStem = 'GroupROBOT';       
    end
    methods (Access = public)
%% Constructor 
        function self = GroupROBOT(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end            

            % Overiding the default workspace for this small robot
            self.workspace = [-0.6 0.6 -0.6 0.6 -0.01 0.8];   

            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link([0      0.29       0         -pi/2  0]);
            link(2) = Link([-pi/2  0          0.27      0      0]);
            link(3) = Link([0      0          0.07      -pi/2  0]);
            link(4) = Link([0      0.302      0         pi/2   0]);
            link(5) = Link([0      0          0         -pi/2  0]);
            link(6) = Link([0      0.072      0          0     0]);

            % Incorporate joint limits
            link(1).qlim = [-165 165]*pi/180;
            link(2).qlim = [-110 110]*pi/180;
            link(3).qlim = [-110 70]*pi/180;
            link(4).qlim = [0 0]*pi/180;
            link(5).qlim = [-120 120]*pi/180;
            link(6).qlim = [-400 400]*pi/180;

            link(2).offset =  -pi/2;
            link(5).offset =  pi/2;

            self.model = SerialLink(link,'name',self.name); 
        end    
    end
end