classdef UR3eControl < handle
    properties
        robot            % UR3e robot instance
        workspace        % Workspace boundaries for the simulation
        cubeInstance     % Instance of the cube class
    end
    
    %% Constructor and Initialization Methods
    methods
        function obj = UR3eControl()
            % Constructor: Initializes the simulation environment, robot, and cubes.
            obj.initializeEnvironment();
            obj.setupRobot();
            obj.createCubes(); % Create cubes in the environment
        end

        function initializeEnvironment(obj)
            % Initializes the workspace and sets up the environment and robot.
            disp('Initializing the workspace environment...');
            obj.workspace = [-1 1 -1 1 -0.05 1]; % Define workspace size
            % Set up environment
            obj.setupEnvironment();
        end
    end
    
    %% Cube Handling Method
    methods
        function createCubes(obj)
            % Creates cubes in the workspace using the cube class
            disp('Creating cubes in the workspace...');
            obj.cubeInstance = cube(3); % Create 3 cubes (can change the number)
        end
    end
    
    %% Environment Setup Methods
    methods
        function setupEnvironment(obj)
            % Configures the environment for the simulation.
            clf; % Clear the current figure
            disp('Setting up the environment...');
            hold on; % Retain current plot when adding new elements
            axis([-1, 1, -1, 1, 0, 1]);  % Set appropriate axis bounds for the robot
            view(3); % Set the view to 3D
            xlabel('X-axis'); % Label x-axis
            ylabel('Y-axis'); % Label y-axis
            zlabel('Z-axis'); % Label z-axis
            title('UR3e Simulation'); % Set plot title
        end
    end
    
    %% Robot Setup Methods
    methods
        function setupRobot(obj)
            % Sets up the UR3e robot model in the simulation environment.
            baseTransform = eye(4); % Define base transformation
            obj.robot = UR3e(baseTransform); % Initialize the robot with the base transform
            initialJointAngles = [pi, -pi/2, 0, 0, 0, 0];
            obj.moveToJointStates(initialJointAngles); % Move robot to initial joint angles
            assignin('base', 'ur3Robot', obj.robot); % Assign robot to base workspace
        end
    end
    
    %% Robot Motion and Pose Methods
    methods
        function moveToJointStates(obj, jointStates)
            % Moves the robot to the specified joint states.
            
            disp('Moving robot to specified joint states...');
            if isvector(jointStates) && length(jointStates) == obj.robot.model.n
                obj.robot.model.plot(jointStates); % Plot robot at specified joint states
                drawnow(); % Update the plot
                disp('Robot moved to joint states.');
            else
                error('Invalid joint states input. Expected %d joint states.', obj.robot.model.n);
            end
        end
                
        function checkPose(obj, targetPose)
            % Checks if the robot's end effector is within ±5mm of the target pose.
            currentPose = obj.robot.model.fkine(obj.robot.model.getpos()); % Compute current pose
            distance = norm(currentPose(1:3, 4) - targetPose(1:3, 4)); % Calculate distance
            
            if distance <= 0.005
                disp('End effector is within ±5mm of the target pose.');
            else
                disp('End effector is NOT within ±5mm of the target pose.');
            end
        end
    end
end
