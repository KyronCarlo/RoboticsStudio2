classdef cube
    properties
        cubeModel % Cell array to store cube model objects
        cubeCount % Total number of cubes to be created
        boundary % Boundaries for random placement [xMin xMax yMin yMax zMin zMax]
        cubeSize % Size of each cube
        cubeColors % Colors for each cube
        locations % Store locations of each cube
    end
    
    methods
        function obj = cube(cubeCount, varargin)
            % Constructor: Initialize cubes with optional parameters
            % Usage: myCubes = cube(5) - Creates 5 cubes with default settings
            % Usage: myCubes = cube(5, [0 0.5 -0.3 0.3 0 0.2], 0.03) - Creates 5 cubes within specified boundary and size
            
            obj.cubeCount = cubeCount;
            obj.cubeModel = cell(1, cubeCount);
            obj.locations = zeros(cubeCount, 3); % Initialize locations array
            
            % Default boundary if not specified [xMin xMax yMin yMax zMin zMax]
            if nargin > 1 && ~isempty(varargin{1})
                obj.boundary = varargin{1};
            else
                obj.boundary = [-0.3 0.3 -0.3 0.3 0 0.2]; % Default boundary
            end
            
            % Default cube size if not specified
            if nargin > 2 && ~isempty(varargin{2})
                obj.cubeSize = varargin{2};
            else
                obj.cubeSize = 0.025; % Default size (25mm)
            end
            
            % Generate random colors for cubes - one color per cube
            obj.cubeColors = rand(cubeCount, 3);
            
            % Create and place each cube
            obj.CreateCubes();
            
            % Print the locations of all cubes
            obj.PrintLocations();
        end
        
        function CreateCubes(obj)
            % Create cubes and place them at random locations within boundary
            for i = 1:obj.cubeCount
                % Generate random position within boundary
                x = obj.boundary(1) + (obj.boundary(2) - obj.boundary(1)) * rand();
                y = obj.boundary(3) + (obj.boundary(4) - obj.boundary(3)) * rand();
                z = obj.boundary(5) + (obj.boundary(6) - obj.boundary(5)) * rand();
                
                % Store the location
                obj.locations(i,:) = [x, y, z];
                
                % Create a cube mesh
                [f, v] = obj.CreateCubeMesh(obj.cubeSize);
                
                % Create a cube object
                obj.cubeModel{i} = obj.PlotCube(f, v, [x, y, z], obj.cubeColors(i,:));
                
                % Check for collisions and reposition if needed
                if i > 1
                    collisionDetected = true;
                    attempts = 0;
                    maxAttempts = 50;
                    
                    while collisionDetected && attempts < maxAttempts
                        collisionDetected = false;
                        
                        % Check collision with previous cubes
                        for j = 1:i-1
                            % Get positions
                            pos1 = obj.cubeModel{i}.base(1:3, 4)';
                            pos2 = obj.cubeModel{j}.base(1:3, 4)';
                            
                            % Calculate distance between centers
                            distance = norm(pos1 - pos2);
                            
                            % If too close, consider it a collision
                            if distance < (obj.cubeSize * 2)
                                collisionDetected = true;
                                break;
                            end
                        end
                        
                        % If collision, reposition the cube
                        if collisionDetected
                            x = obj.boundary(1) + (obj.boundary(2) - obj.boundary(1)) * rand();
                            y = obj.boundary(3) + (obj.boundary(4) - obj.boundary(3)) * rand();
                            z = obj.boundary(5) + (obj.boundary(6) - obj.boundary(5)) * rand();
                            obj.cubeModel{i}.base = SE3(x, y, z);
                            obj.locations(i,:) = [x, y, z]; % Update stored location
                            attempts = attempts + 1;
                        end
                    end
                    
                    % If max attempts reached, place it anyway
                    if attempts >= maxAttempts
                        disp(['Warning: Could not find non-colliding position for cube ', num2str(i)]);
                    end
                    
                    % Update the final position in the locations array
                    obj.locations(i,:) = obj.cubeModel{i}.base(1:3, 4)';
                end
            end
        end
        
        function PrintLocations(obj)
            % Print the locations of all cubes to the terminal
            disp('=== Cube Locations ===');
            for i = 1:obj.cubeCount
                location = obj.cubeModel{i}.base(1:3, 4)';
                disp(['Cube ', num2str(i), ': [', num2str(location(1), '%.4f'), ', ', ...
                      num2str(location(2), '%.4f'), ', ', num2str(location(3), '%.4f'), ']']);
            end
            disp('=====================');
        end
        
        function UpdateLocation(obj, cubeIndex, newPosition)
            % Update the location of a specific cube
            if cubeIndex <= obj.cubeCount && cubeIndex > 0
                obj.cubeModel{cubeIndex}.base = SE3(newPosition);
                obj.locations(cubeIndex,:) = newPosition;
                
                % Print the updated location
                disp(['Updated Cube ', num2str(cubeIndex), ' location: [', ...
                      num2str(newPosition(1), '%.4f'), ', ', ...
                      num2str(newPosition(2), '%.4f'), ', ', ...
                      num2str(newPosition(3), '%.4f'), ']']);
            else
                disp('Invalid cube index');
            end
        end
        
        function [faces, vertices] = CreateCubeMesh(~, size)
            % Create a cube mesh with specified size
            s = size/2; % Half-size for vertices from center
            
            % Define vertices of a cube centered at origin
            vertices = [
                -s -s -s;  % 1 - bottom face, left front
                 s -s -s;  % 2 - bottom face, right front
                 s  s -s;  % 3 - bottom face, right back
                -s  s -s;  % 4 - bottom face, left back
                -s -s  s;  % 5 - top face, left front
                 s -s  s;  % 6 - top face, right front
                 s  s  s;  % 7 - top face, right back
                -s  s  s   % 8 - top face, left back
            ];
            
            % Define faces (triangles) using vertices
            faces = [
                1 2 3;  % Bottom face (triangle 1)
                1 3 4;  % Bottom face (triangle 2)
                5 6 7;  % Top face (triangle 1)
                5 7 8;  % Top face (triangle 2)
                1 2 6;  % Front face (triangle 1)
                1 6 5;  % Front face (triangle 2)
                2 3 7;  % Right face (triangle 1)
                2 7 6;  % Right face (triangle 2)
                3 4 8;  % Back face (triangle 1)
                3 8 7;  % Back face (triangle 2)
                4 1 5;  % Left face (triangle 1)
                4 5 8   % Left face (triangle 2)
            ];
        end
        
        function cubeObj = PlotCube(~, faces, vertices, position, color)
            % Create a plotted cube object at specified position with given color
            % Create a figure handle if none exists
            if isempty(findobj('Type', 'figure'))
                figure;
                hold on;
                grid on;
                axis equal;
            end
            
            % Plot cube with specified color
            h = patch('Faces', faces, 'Vertices', vertices, 'FaceColor', color, ...
                'EdgeColor', 'k', 'FaceAlpha', 0.8);
            
            % Create a rigidBody object for the cube
            cubeObj = rigidBody('cube');
            cubeObj.base = SE3(position);
            
            % Add plot handle to use for animation
            cubeObj.h = h;
        end
    end
    
    methods (Static)
        function animate(cubeObj, q)
            % Animate the cube
            % This function takes a cube object and a joint configuration q
            % It updates the cube's position based on its base transformation
            
            if ~isempty(cubeObj) && isfield(cubeObj, 'h') && ishandle(cubeObj.h)
                % Get current vertices
                vertices = get(cubeObj.h, 'Vertices');
                
                % Get current center
                currentCenter = mean(vertices);
                
                % Get new position from base transformation
                newPosition = cubeObj.base.t;
                
                % Calculate translation needed
                translation = newPosition' - currentCenter;
                
                % Apply translation to all vertices
                newVertices = vertices + translation;
                
                % Apply rotation if needed (part of the base transformation)
                R = cubeObj.base.R;  % Rotation matrix from base
                if ~isequal(R, eye(3))
                    % Center vertices at origin, rotate, then translate back
                    centeredVertices = newVertices - newPosition';
                    rotatedVertices = (R * centeredVertices')';
                    newVertices = rotatedVertices + newPosition';
                end
                
                % Update the cube's vertices
                set(cubeObj.h, 'Vertices', newVertices);
                drawnow();
                
                % Print the updated position to terminal
                disp(['Cube moved to: [', num2str(newPosition(1), '%.4f'), ', ', ...
                      num2str(newPosition(2), '%.4f'), ', ', num2str(newPosition(3), '%.4f'), ']']);
            end
        end
    end
end