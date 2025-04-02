classdef UR3eMotionPlanning < handle
    properties
        robot            % UR3e robot instance
        workspace        % Workspace boundaries for the simulation
        cubeInstance     % Instance of the cube class
    end
    
    %% Constructor and Initialization Methods
    methods
        function obj = UR3eMotionPlanning()
            % Constructor: Initializes the simulation environment, robot, and cubes.
            obj.initializeEnvironment();
            obj.setupRobot();
            obj.createCubes(); % Create cubes in the environment

            % First move to the safe position
            obj.moveToSafePosition();

            % Call the function to print joint angles automatically
            obj.printJointAnglesAboveCubes();
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
            obj.robot = UR3e(baseTransform); % Initialize the robot with the base transform\

            qInitial = [0, -pi/2, 0, 0, 0, 0];
            
            obj.robot.model.plot(qInitial, 'workspace', obj.workspace); % Plot the robot in the workspace
            hold on;
            drawnow(); % Update the plot
            disp('Robot initialized at the initial joint angles.');
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

       function moveToSafePosition(obj)
            % Moves the robot to a predefined safe position
            % Safe position joint angles in degrees: [-61.26 -81.48 -92.51 -91.86 85.49 6.96]
            
            disp('Moving robot to safe position...');
            
            % Define safe position joint angles in degrees
            safeJointAnglesDeg = [-61.26, -81.48, -92.51, -91.86, 85.49, 6.96];
            
            % Convert from degrees to radians for the robot model
            safeJointAnglesRad = deg2rad(safeJointAnglesDeg);
            
            % Call the existing moveToJointStates function to move the robot
            obj.moveToJointStates(safeJointAnglesRad);
            
            disp('Robot moved to safe position.');
            
            % Display the safe position joint angles for reference
            disp('Safe position joint angles (degrees):');
            disp(['  [', num2str(safeJointAnglesDeg, '%.2f '), ']']);
            disp('Safe position joint angles (radians):');
            disp(['  [', num2str(safeJointAnglesRad, '%.4f '), ']']);
       end

       function printJointAnglesAboveCubes(obj)          
            % Prints the joint angles required to position the end effector 20cm above each cube
            % This function calculates but does not execute the movement
            
            disp('Calculating joint angles for positions 20cm above each cube...');
            
            % Access cube positions from the cube instance
            cubePositions = obj.cubeInstance.cubePositions;
            numCubes = size(cubePositions, 1);
            
            % Get current joint angles as initial guess for ikcon
            currentJointAngles = obj.robot.model.getpos();
            
            % Loop through each cube
            for i = 1:numCubes
                % Get cube position
                cubePos = cubePositions(i, :);
                
                % Calculate target position 20cm above the cube
                targetPosition = cubePos + [0, 0, 0.2]; % Add 20cm in Z direction
                
                % Create target transformation matrix
                % Using standard down-facing orientation for the end effector
                targetRotation = rpy2tr(0, pi, 0); % End effector facing down
                targetTransform = transl(targetPosition) * targetRotation;
                
                % Use ikcon to calculate joint angles for the target transform
                [jointAngles, ~, ~] = obj.robot.model.ikcon(targetTransform, currentJointAngles);
                
                % Display the joint angles
                disp(['Joint angles for position 20cm above Cube ', num2str(i), ':']);
                disp(['  Joint 1: ', num2str(jointAngles(1), '%.4f')]);
                disp(['  Joint 2: ', num2str(jointAngles(2), '%.4f')]);
                disp(['  Joint 3: ', num2str(jointAngles(3), '%.4f')]);
                disp(['  Joint 4: ', num2str(jointAngles(4), '%.4f')]);
                disp(['  Joint 5: ', num2str(jointAngles(5), '%.4f')]);
                disp(['  Joint 6: ', num2str(jointAngles(6), '%.4f')]);
            end
       end
    end            
end
