classdef cube < handle
    %   CUBES A class that creates a collection of cubes.
    %   The cubes can be moved around randomly. It is then possible to query
    %   the current location (base) of the cubes.    
    
    %#ok<*TRYNC>    

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 0.00;
    end
    
    properties
        %> Number of cubes
        cubeCount = 3;  % Set to 3 cubes
        
        %> A cell structure of cubeCount cube models
        cubeModel;
        
        %> Dimensions of the workspace in regard to the paddock size
        workspaceDimensions = [-0.2, 0.2, 0.17, 0.425, 0, 0.1]; % Set workspace dimensions for cubes
    end
    
    methods
        %% Constructor
        function self = cube(cubeCount)
            if nargin > 0
                self.cubeCount = cubeCount;
            end

            % Store the positions of the cubes to avoid overlap
            cubePositions = zeros(self.cubeCount, 2);  % Only x and y positions are needed for comparison
            
            % Create the required number of cubes
            for i = 1:self.cubeCount
                self.cubeModel{i} = self.GetCubeModel(['cube', num2str(i)]); % Get the cube model
                
                % Initialize overlap check
                overlap = true;
                while overlap
                    % Random spawn within the defined workspace
                    xPos = rand * (self.workspaceDimensions(2) - self.workspaceDimensions(1)) + self.workspaceDimensions(1);
                    yPos = rand * (self.workspaceDimensions(4) - self.workspaceDimensions(3)) + self.workspaceDimensions(3);
                    zPos = 0;

                    % Check for overlap with existing cubes
                    overlap = false;
                    for j = 1:i-1
                        % Calculate distance between the current and previous cubes
                        distance = sqrt((xPos - cubePositions(j, 1))^2 + (yPos - cubePositions(j, 2))^2);
                        
                        % If distance is too small (overlap), generate new position
                        if distance < 0.1  % You can adjust this threshold based on the size of the cubes
                            overlap = true;
                            break;
                        end
                    end
                end

                % Store the new valid position
                cubePositions(i, :) = [xPos, yPos];

                % Create the base pose for the cube with valid position
                basePose = transl(xPos, yPos, zPos);  % Create random position for each cube
                % Print the position of the cube to the terminal
                disp(['Cube ', num2str(i), ' position: ', 'X: ', num2str(xPos), ', Y: ', num2str(yPos), ', Z: ', num2str(zPos)]);

                self.cubeModel{i}.base = basePose;  % Assign base position
                
                % Plot the 3D model of the cube
                plot3d(self.cubeModel{i}, 0, 'workspace', self.workspaceDimensions, 'view', [-30, 30], 'delay', 0, 'noarrow', 'nowrist');
                
                % Hold on after the first plot (if already on there's no difference)
                if i == 1
                    hold on;
                end
            end

            axis equal;
            if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
                camlight;
            end
        end
        
    end
    
    methods (Static)
        %% GetCubeModel
        function model = GetCubeModel(name)
            if nargin < 1
                name = 'Cube';
            end
            [faceData, vertexData] = plyread('cube.ply', 'tri');  % Load the ply file for the cube
            % Scale factor
            scaleFactor = 0.02;  % Scale down
            
            % Apply scaling to the vertices
            vertexData = vertexData * scaleFactor;
            
            % Create the model using the scaled vertices
            link1 = Link('alpha', 0, 'a', 0, 'd', 0.02, 'offset', 0);
            model = SerialLink(link1, 'name', name);
            
            % Changing the order of the data so it gets attributed to the correct link in plot3d
            model.faces = {[], faceData};
            model.points = {[], vertexData};  % Use the scaled vertex data
        end
    end    
end
