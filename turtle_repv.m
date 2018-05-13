%==========================================================================

% MATLAB code for Project 3 (Planning Class)
% Turtlebot Path Planning using A* Algorithm
%
% Written by Yash Shah (115710498)
%
% email ID: ysshah95@umd.edu
% 
% Implementation of A* Algorithm for finding a shortest path 
% between two points in the given environment (with obstacles) for
% Turtlebot2

%==========================================================================

% clc
% clear all

% Load the path file found using previous program
load('Path.mat','path');

% Initialize Vrep Communication
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% initialize velocity
linear.x = 0;
linear.y = 0;
linear.z = 0;

angular.x = 0;
angular.y = 0;
angular.z = 0;

fileID = fopen('velocity.txt','w');
fprintf(fileID,'%6s %12s %18s %24s %30s %36s %42s\r\n','TimeStamp','x_dot',...
    'y_dot', 'z_dot', 'alpha_dot', 'beta_dot', 'gamma_dot');

% Run the following algorithm only if Connection is Established with Vrep
if (clientID>-1)
    
    disp('Connected to Vrep');
    
    % Get Object Handles from Vrep
    
    [~,Robot]=vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking)  ;  
    [~,left_Motor]=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking);
    [~,right_Motor]=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking);
    
    % Get the position and Angle of the Robot from Vrep
    
    [~,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
    [~,angle]=vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_blocking);

    % Import the Path generated from A* Algorithm and reorient it so that
    % it can be used for this environment 
    
    path = flipud(path);
    Pathx = path(:,1);
    Pathy = path(:,2);
    
    k = length(path);
    
    % Initialize the Current and Target Position
    
    current_pos =[round(position(1,1),1),round(position(1,2),1)];
    target_pos = [Pathx(k),Pathy(k)];

    radius = 0.038;
    dist_wheels=0.3175;
    i = 2;
    j = 1;
    tic

    while norm(current_pos - target_pos) > 0.2
    
        % Get the coordinates of the next point
        next_pos = [Pathx(i),Pathy(i)];
        i = i+1 ;

        % Calculate the slope of current position and next position
        slope = atan2(round((next_pos(2)-current_pos(2)),1),round((next_pos(1) -current_pos(1)),1));
    
        % Transform the slope Variable such that it matched the angle
        % configuration recieved from the Vrep
        
        if slope > 3.135 | slope < -3.135
            slope = 0;
        else
            slope = slope + 3.14;
        end
        
        % Get the angle of the Robot in Real Time from Vrep

        [~,angle]=vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_blocking);
        
        % Calculate the velocity of both weheels
        
        time = 0.5;
        vel_right = ((slope - angle(3))*dist_wheels)/(2*radius*time);
        % SInce we just have to rotate the robot, we use equal and opposite
        % velocity for the other wheel.
        vel_left = -vel_right;
        while abs(vel_right) > 2 
            time = time + 0.5 ;
            vel_right = ((slope - angle(3))*dist_wheels)/(2*radius*time);
            vel_left = -vel_right;
        end
        
        % Save the velocities to the txt file
        
        timeStamp = toc;
        [x_dot y_dot theta_dot] = extract_velocity(vel_right, vel_left, slope);
        line = [timeStamp x_dot y_dot linear.z theta_dot angular.y angular.z];
        fprintf(fileID,'%6.2f %12.2f %18.2f %24.2f %30.2f %36.2f %42.2f\r\n', line);
        
        % Rotate the robot with the calculated velocities till the angle is
        % equal to the slope between currwnt position and next position 
        while (abs((slope) - angle(3))) > 0.04
            [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,vel_left,vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,vel_right,vrep.simx_opmode_blocking);
            [~,angle]=vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_blocking);
        end
        
        % Stop the rotation 
        [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
        
        % Get the Position of the Robot 
        [~,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
        current_pos = [position(1),position(2)]; 
        
        % Claculate the velocity based on the distance between two points
        time = 0.05;
        vel_right = (norm(next_pos -current_pos))/time;
        % Since we just have to translate the robot straight, we use equal 
        % velocity for the other wheel.
        vel_left=vel_right;
                
        % Make sure that the velocity obtained is less than 8rad/sec. 
        while abs(vel_right) > 8 
            time = time + 0.01 ;
            vel_right = (norm(next_pos -current_pos))/time;
            vel_left = vel_right;
        end
        
        % Save the velocities to the txt file. 
        timeStamp = toc;
        [x_dot y_dot theta_dot] = extract_velocity(vel_right, vel_left, slope);
        line = [timeStamp x_dot y_dot linear.z theta_dot angular.y angular.z];
        fprintf(fileID,'%6.2f %12.2f %18.2f %24.2f %30.2f %36.2f %42.2f\r\n', line);
        
        % Get the Position of the Robot from VRep
        [~,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
        current_pos = [position(1),position(2)];    

        % Translate the robot with the calculated velocities till the 
        % current position is equal to the next position.
        while ((norm(next_pos -current_pos)) > 0.09)
%             norm(next_pos -current_pos)
            [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,vel_left,vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,vel_right,vrep.simx_opmode_blocking);
            [~,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
            current_pos = [position(1),position(2)];
        end
        
        % Stop the Translation. 
        [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);

        % Update the Current Position of the for the future use.  
        [~,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
        current_pos = [position(1),position(2)];

    end
    % End the Vrep Communication 
    vrep.simxFinish(-1);
end

vrep.delete();
% Close the txt file. 
fclose(fileID);