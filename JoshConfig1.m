function finalRad= ExampleControlProgram(serPort)
% Simple program for autonomously control the iRobot Create on either the
% physical Create or the simulated version. This will simply spiral outward
% and turn away from obstacles that detects with the bump sensors.
%
% For the physical Create only, it is assumed that the function call
% serPort= RoombaInit(comPort) was done prior to running this program.
% Calling RoombaInit is unnecessary if using the simulator.
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% finalRad - Double, final turning radius of the Create (m)

% ExampleControlProgram.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com

    % Set constants for this program
    maxDuration= 12000;  % Max time to allow the program to run (s)
    maxDistSansBump= 5; % Max distance to travel without obstacles (m)
    maxFwdVel= 0.5;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
    maxVelIncr= 0.005;  % Max incrementation of forward velocity (m/s)
    maxOdomAng= pi/4;   % Max angle to move around a circle before 
                        % increasing the turning radius (rad)
    
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    angTurned= 0;       % Angle turned since turning radius increase (rad)
    v= 0.3;               % Forward velocity (m/s)
    w= 0.0;          % Angular velocity (rad/s)
    
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w)
    
    % Enter main loop
    while toc(tStart) < maxDuration && distSansBump <= maxDistSansBump
        distSansBump= distSansBump+DistanceSensorRoomba(serPort);
        angTurned= angTurned+AngleSensorRoomba(serPort);

% Below if-loop is for moving the bot for a specific amount of time         
%         if toc(tStart) >= 8
%             w = 0.4;
%             SetFwdVelAngVelCreate(serPort,v,w)
%             if toc(tStart)>=12
%                 w = -0.4;
%                 SetFwdVelAngVelCreate(serPort,v,w)
%             end
%         end
%
    
%  Below is the very beginning of an avoidance system
%  only works when the bot is running into a wall, and is at an angle
%  to the wall, such that the right beam can see the wall 
            sonarArray = [ReadSonar(serPort, 2) ReadSonar(serPort, 1) ReadSonar(serPort, 3) ReadSonar(serPort,4 )];
            if any(sonarArray<= 0.2)
               SetFwdVelAngVelCreate(serPort,0,0)
               if sonarArray(2)<3
                   turnParallelWall(serPort, sonarArray(1), sonarArray(2))
               elseif sonarArray(3)<3
                   turnParallelWall(serPort, sonarArray(1), sonarArray(3))
               else
                   turnAngle(serPort, 0.2, 45)
               end
            end
        % Briefly pause to avoid continuous loop iteration
        pause(0.1)
    end
    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w)
     
%     % If you call RoombaInit inside the control program, this would be a
%     % good place to clean up the serial port with...
%     % fclose(serPort)
%     % delete(serPort)
%     % clear(serPort)
%     % Don't use these if you call RoombaInit prior to the control program
end

function turnParallelWall(serPort, sonarFront, sonarRight)
  [angWall1 angWall2 wallLength] = triangWall(sonarFront, sonarRight);
  pause(0.2);
  turnAngle(serPort, 0.2, angWall2)
  pause(0.1);
  SetFwdVelAngVelCreate(serPort,0.4,0)
end

function [angB angC wallLength] = triangWall(sensorC, sensorB)
%triangWall uses two sonar senors, which are placed 90deg apart, to deduce
%the angle of the bot in relation to the wall 
    wallLength = sqrt(sensorC.^2 + sensorB.^2);
    angB = asind(sensorC/wallLength);
    angC = asind(sensorB/wallLength);
end
    
    
    



function w= v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    
    % Robot constants
    maxWheelVel= 2.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.6;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end