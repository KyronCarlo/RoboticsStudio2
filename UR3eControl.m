classdef UR3eControl
    properties
        UR3e % robotSim for the UR3e robot model
        TotalCubes = 3 
        Steps = 40 % Changes speed of animation
        % Cube parameters
        cubeBoundary % Boundary for cube placement
        cubeSize % Size of each cube
    end
    
    methods
        function robotSim = UR3eControl(totalCubes, cubeBoundary, cubeSize)
            % Constructor: Initialize environment, robot, and simulate cube handling
            % Usage: myRobot = UR3eControl() - Creates control with default cube settings
            % Usage: myRobot = UR3eControl(5) - Creates control with 5 cubes
            % Usage: myRobot = UR3eControl(5, [0 0.5 -0.3 0.3 0 0.2]) - Specifies boundary too
            % Usage: myRobot = UR3eControl(5, [0 0.5 -0.3 0.3 0 0.2], 0.03) - Specifies size too
            
            % Set cube count if provided
            if nargin > 0 && ~isempty(totalCubes)
                robotSim.TotalCubes = totalCubes;
            end
            
            % Set cube boundary if provided
            if nargin > 1 && ~isempty(cubeBoundary)
                robotSim.cubeBoundary = cubeBoundary;
            else
                robotSim.cubeBoundary = [-0.3 0.3 -0.3 0.3 0 0.2]; % Default boundary
            end
            
            % Set cube size if provided
            if nargin > 2 && ~isempty(cubeSize)
                robotSim.cubeSize = cubeSize;
            else
                robotSim.cubeSize = 0.025; % Default size (25mm)
            end
            
            robotSim.InitializeEnvironment(); 
            robotSim.UR3e = UR3e(); 
            robotSim.SetupRobot(); 
            robotSim.SimulateCubeHandling(); 
            robotSim.CalculateWorkspaceVolume(); 
        end

        function InitializeEnvironment(robotSim)
            % Setup the simulation environment and place robotSimects
            clear figure; 
            hold on; 
            grid on; 
            axis equal; % Set axis scaling to equal
            axis([-6, 6, -6, 6, -1, 4]); % Define axis limits (X, Y, Z)
            view(3); % Set view to 3D perspective
            
            % Place table in the workspace
            table = robotSim.PlacerobotSim('tableBrown2.1x1.4x0.5m.ply', [0, 0, -0.5]);
            verts = [get(table, 'Vertices'), ones(size(get(table, 'Vertices'), 1), 1)] * trotz(pi/2); % Rotate table by 90 degrees
            verts(:, 1) = verts(:, 1) * 2; % Stretch table in x direction
            set(table, 'Vertices', verts(:, 1:3));
        end

        function h = PlacerobotSim(~, fileName, position)
            % Place robotSim in the workspace at a given position
            [f, v, data] = plyread(fileName, 'tri'); % Read the robotSim data from PLY file
            % Plot with improved rendering quality
            h = trisurf(f, v(:, 1), v(:, 2), v(:, 3), ...
                'FaceVertexCData', [data.vertex.red, data.vertex.green, data.vertex.blue] / 255, ...
                'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'SpecularStrength', 0.5);

            h.Vertices = bsxfun(@plus, h.Vertices, position); % Translate robotSim to the specified position
        end

        function Cube = InitializeCubes(robotSim, totalCubes)    
            % Initialize cubes with the specified parameters
            Cube = cube(totalCubes, robotSim.cubeBoundary, robotSim.cubeSize);
        end

        function robotSim = SetupRobot(robotSim)
            % Configure the UR3e robot and set up the workspace visualization
            UR3eBase = eye(4); % 4x4 identity matrix for the base transformation
            robotSim.UR3e.model.base = robotSim.UR3e.model.base.T * UR3eBase * trotz(pi/2) * trotx(pi/2) * transl([0, 0, -0.5]);
            
            workspace = [-1 1 -1 1 -0.5 1]; % Define the workspace boundaries
            scale = 0.5; % Scale factor for better visualization
            
            qInitial = [0 pi 0 0 0 0 0]; % Initial joint angles of the robot
            robotSim.UR3e.model.plot(qInitial, 'workspace', workspace, 'scale', scale); % Plot the robot in the workspace
            hold on;
        end
       
        function SimulateCubeHandling(robotSim)
            % Simulate the process of handling cubes with the robot
            Cube = robotSim.InitializeCubes(robotSim.TotalCubes); % Initialize cubes
            
            % Define the goal pose and compute the trajectory to reach it
            qCurrent = robotSim.UR3e.model.getpos();
            goalPose = transl(-0.2, -0.2, 0.2);
            qFinal = robotSim.UR3e.model.ikine(goalPose, 'qInitial', robotSim.UR3e.model.getpos(), 'mask', [1 1 1 0 0 0]);
            
            qPath = jtraj(qCurrent, qFinal, robotSim.Steps);

            disp('Joint angles at goal Pose: ');
            disp(qFinal);

            % Animate the robot moving to the goal pose
            for i = 1:robotSim.Steps
                robotSim.UR3e.model.animate(qPath(i, :));
            end    
            
            % Loop through each cube to pick up and place it
            for j = 1:robotSim.TotalCubes
                qCurrent = robotSim.UR3e.model.getpos();
                qFinal = robotSim.UR3e.model.ikcon(Cube.cubeModel{j}.base);
                qPath = jtraj(qCurrent, qFinal, robotSim.Steps);  
     
                disp("Moving to cube");
                % Animate the robot moving to the cube
                for i = 1:robotSim.Steps
                    robotSim.UR3e.model.animate(qPath(i, :));
                end
                     
                disp("Picking up cube");  
                % Get and display the current transformation matrix
                currentTransform = robotSim.UR3e.model.fkine(qPath(i, :));
                disp(' Current Transform:');
                disp(currentTransform);
                
                qCurrent = robotSim.UR3e.model.getpos();
                qFinal = [0.6687    0.7250   -0.8125   -0.7610   -0.6312    0.4043         0]; %goalPose joint angles
               
                qPath = jtraj(qCurrent, qFinal, robotSim.Steps);
                visualAdjustment = SE3(0, 0, 0.01); % Adjustment for visual alignment

                % Animate the cube adjustment
                for i = 1:robotSim.Steps
                    robotSim.UR3e.model.animate(qPath(i, :));
                    qPos = robotSim.UR3e.model.fkine(qPath(i, :));
                    Cube.cubeModel{j}.base = qPos * visualAdjustment;
                    animate(Cube.cubeModel{j}, 0);
                end
                
                % Define position for the cube and move the robot to place it
                    qCurrent = qFinal;
                    a = 7.7;
                if j <= 3
                    qFinal = robotSim.UR3e.model.ikcon(transl(-1.000, (-0.180 + j/a), 0.025) * trotx(pi) * trotz(pi) * transl(-0.05));
                elseif j <= 6
                    qFinal = robotSim.UR3e.model.ikcon(transl(-1.000, (-0.180 + (j - 3)/a), 0.060) * trotx(pi) * trotz(pi) * transl(-0.05));
                else
                    qFinal = robotSim.UR3e.model.ikcon(transl(-1.000, (-0.180 + (j - 6)/a), 0.095) * trotx(pi) * trotz(pi) * transl(-0.05));
                end
                 
                    qPath = jtraj(qCurrent, qFinal, robotSim.Steps);
                    disp("Placing the cube");
                    
                    % Animate the robot placing the cube
                for i = 1:robotSim.Steps
                    robotSim.UR3e.model.animate(qPath(i, :));
                    qPos = robotSim.UR3e.model.fkine(qPath(i, :));
                    Cube.cubeModel{j}.base = qPos * visualAdjustment;
                    animate(Cube.cubeModel{j}, 0);
                end
                
                % Set final position of the cube
                if j <= 3
                    bPos = transl(-1.000, (-0.250 + j/a), 0.100) * trotx(pi) * trotz(pi);
                elseif j <= 6
                    bPos = transl(-1.000, (-0.250 + (j - 3)/a), 0.135) * trotx(pi) * trotz(pi);
                else
                    bPos = transl(-1.000, (-0.250 + (j - 6)/a), 0.170) * trotx(pi) * trotz(pi);
                end
                Cube.cubeModel{j}.base = bPos;
                animate(Cube.cubeModel{j}, 0);

                % Get and display the current transformation matrix
                currentTransform = robotSim.UR3e.model.fkine(qPath(i, :));
                disp('Current Transform:');
                disp(currentTransform)
                
                % Return to start pose
                qCurrent = robotSim.UR3e.model.getpos();
                qFinal = [0 pi 0 0 0 0 0];
                qPath = jtraj(qCurrent, qFinal, robotSim.Steps);
                for i = 1:robotSim.Steps
                    robotSim.UR3e.model.animate(qPath(i, :));
                end
            end

            disp("All the cubes have been placed");
        end

        function CalculateWorkspaceVolume(robotSim)
            % Taken from lab code:
            % Calculate and display the volume of the robot's workspace
            stepRads = deg2rad(30); % Step size for joint angle increments
            qlim = robotSim.UR3e.model.qlim; % Joint limits
            pointCloudSize = prod(floor((qlim(1:5, 2) - qlim(1:5, 1)) / stepRads + 1)); % Size of the point cloud
            pointCloud = zeros(pointCloudSize, 3); % Initialize point cloud array
            counter = 1;
            
            % Iterate over joint angles to generate the point cloud  
            for q1 = qlim(1, 1):stepRads:qlim(1, 2)
                for q2 = qlim(2, 1):stepRads:qlim(2, 2)
                    for q3 = qlim(3, 1):stepRads:qlim(3, 2)
                        for q4 = qlim(4, 1):stepRads:qlim(4, 2)
                            for q5 = qlim(5, 1):stepRads:qlim(5, 2)
                                for q6 = qlim(6, 1):stepRads:qlim(6, 2)
                                    q7 = 0; % Assume joint 7 is 0
                                    qFinal = [q1, q2, q3, q4, q5, q6, q7];
                                    tr = robotSim.UR3e.model.fkine(qFinal).T; % Get end-effector transformation matrix
                                    pointCloud(counter, :) = tr(1:3, 4)'; % Store position in point cloud
                                    counter = counter + 1;
                                end
                            end
                        end
                    end
                end
            end
            
            % Plot the workspace
            plot3(pointCloud(:, 1), pointCloud(:, 2), pointCloud(:, 3), 'r.');
            center = mean(pointCloud); % Compute the center of the point cloud
            distances = sqrt(sum((pointCloud - center) .^ 2, 2)); % Calculate distances from center
            radius = max(distances); % Determine the maximum distance (radius)
            volume = (4/3) * pi * radius^3; % Calculate the volume of the workspace
            
            disp(['Workspace Volume: ', num2str(volume)]);
        end       
    end
end