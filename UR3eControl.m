classdef UR3eControl
    properties
        Robot
        CubesObj
        TargetLocation
    end
    
    methods
        function obj = UR3eControl(numCubes, cubeSize, workspaceLimits, targetLocation)
            obj.Robot = loadrobot('universalUR3e', 'DataFormat', 'column');
            obj.CubesObj = Cubes(numCubes, cubeSize, workspaceLimits);
            obj.TargetLocation = targetLocation;
        end
        
        function executeTask(obj)
            figure;
            obj.CubesObj.plotCubes();
            hold on;
            
            for i = 1:obj.CubesObj.NumCubes
                pickPos = obj.CubesObj.CubePositions(i, :);
                disp(['Picking cube ', num2str(i), ' at ', mat2str(pickPos)]);
                
                qPick = obj.ur3ik(pickPos);
                obj.Robot.plot(qPick');
                pause(1);
                
                qPlace = obj.ur3ik(obj.TargetLocation);
                disp(['Placing cube ', num2str(i), ' at ', mat2str(obj.TargetLocation)]);
                obj.Robot.plot(qPlace');
                pause(1);
            end
            hold off;
            disp('Task completed.');
        end
        
        function q = ur3ik(obj, targetPos)
            ik = inverseKinematics('RigidBodyTree', obj.Robot);
            weights = [1 1 1 1 1 1];
            initialGuess = obj.Robot.homeConfiguration;
            targetPose = trvec2tform(targetPos);
            [qSol, ~] = ik('tool0', targetPose, weights, initialGuess);
            q = qSol;
        end
    end
end
