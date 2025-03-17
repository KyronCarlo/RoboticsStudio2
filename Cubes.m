classdef Cubes
    properties
        NumCubes
        CubeSize
        WorkspaceLimits % [xmin, xmax; ymin, ymax; zmin, zmax]
        CubePositions
    end
    
    methods
        function obj = Cubes(numCubes, cubeSize, workspaceLimits)
            obj.NumCubes = numCubes;
            obj.CubeSize = cubeSize;
            obj.WorkspaceLimits = workspaceLimits;
            obj.CubePositions = obj.generateCubes();
        end
        
        function positions = generateCubes(obj)
            positions = zeros(obj.NumCubes, 3);
            maxAttempts = 1000;
            count = 0;
            
            for i = 1:obj.NumCubes
                attempts = 0;
                while attempts < maxAttempts
                    newPos = [randInRange(obj.WorkspaceLimits(1, :)), ...
                              randInRange(obj.WorkspaceLimits(2, :)), ...
                              obj.WorkspaceLimits(3, 1)]; % Fixed z to avoid stacking
                    
                    if ~obj.isOverlapping(newPos, positions, i-1)
                        positions(i, :) = newPos;
                        break;
                    end
                    attempts = attempts + 1;
                end
                if attempts == maxAttempts
                    warning('Max attempts reached, could not place all cubes.');
                    positions = positions(1:i-1, :);
                    break;
                end
                count = count + 1;
            end
        end
        
        function flag = isOverlapping(obj, newPos, positions, numPlaced)
            flag = false;
            for j = 1:numPlaced
                dist = norm(newPos - positions(j, :));
                if dist < obj.CubeSize
                    flag = true;
                    return;
                end
            end
        end
        
        function plotCubes(obj)
            figure;
            hold on;
            axis equal;
            xlim(obj.WorkspaceLimits(1, :));
            ylim(obj.WorkspaceLimits(2, :));
            zlim(obj.WorkspaceLimits(3, :));
            
            for i = 1:size(obj.CubePositions, 1)
                drawCube(obj.CubePositions(i, :), obj.CubeSize);
            end
            hold off;
        end
    end
end

function val = randInRange(range)
    val = range(1) + (range(2) - range(1)) * rand;
end

function drawCube(position, size)
    [X, Y, Z] = ndgrid([0 1]);
    X = (X - 0.5) * size + position(1);
    Y = (Y - 0.5) * size + position(2);
    Z = (Z - 0.5) * size + position(3);
    
    faces = {[1 2 4 3], [5 6 8 7], [1 2 6 5], [3 4 8 7], [1 3 7 5], [2 4 8 6]};
    hold on;
    for i = 1:length(faces)
        patch(X(faces{i}), Y(faces{i}), Z(faces{i}), 'r', 'FaceAlpha', 0.5);
    end
end
