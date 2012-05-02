function finalRad= ControlProgram2(serPort)

    % git@github.com:/Vroomba/Vroomba.git


    % Set constants for this program
    maxDuration= 999;  % Max time to allow the program to run (s)
    maxDistSansBump= 999; % Max distance to travel without obstacles (m)
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
    global iterateCount;
    iterateCount = 0;
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w)
    stopToken = 0;
    % Enter main loop 
    
    
    % testing turning
    
        % test ccw 1 to 180degrees
        %       turnAngle(serPort, 0.2, 23);
        % test ccw -180 to -360degrees
        %       turnAngle(serPort, 0.2, -190);
        % test cw 0 to -180degrees
               turnAngle(serPort, 0.2, 53);
        % test cw 180 to 360degrees
        %       turnAngle(serPort, 0.2, 190);
        
        stopBot(serPort);
        
    
    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w)

end


function [stopToken] = sonarCheck(serPort, stopToken)
% sonarCheckReact takes two arguments, serPort and a stopToken and performs
% baseline response to a wall detection
    sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
    % iterateCount is for debugging purposes, to allow us to see the
    % program looping
    global iterateCount;    
    if any(sonarArray) < 0.2
       stopBot(serPort)
       sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
       reactToWall(serPort, sonarArray);
    end          
    stopToken = 1;
    iterateCount = iterateCount+1;    
end

function stopBot(serPort)
SetFwdVelAngVelCreate(serPort,0,0)
distTraveled = DistanceSensorRoomba(serPort)
end


function reactToWall(serPort, sonarArray)
% reactToWall takes 2 arguments serPort, and a vector containing all sonar
% readings

    % smallestDist(1) = index of the shortest sonar beams Front or Rear
    % smallestDist(2) = index of the shortest sonar beams Right or Left 
    smallestDist(2) = find(sonarArray == min(sonarArray(2), sonarArray(3)))
    smallestDist(1) = find(sonarArray == min(sonarArray(1), sonarArray(4)))

    if sonarArray(2)>1 && sonarArray(3)>1 && sonarArray(4)>1
        % wall is just front
        turnAngle(serPort, 0.2, 90)
        
    elseif sonarArray(2)>1 && sonarArray(3)>1 && sonarArray(1)>1
        %wall is just rear
        turnAngle(serPort, 0.2, 90)
        
    elseif smallestDist(1) == 1 &&  smallestDist(2) == 2
        % wall is front and right -- must turn ccw
        [angleFB angleRL wallLength] = triangWall(sonarArray(1), sonarArray(2));
        angleToTurn = angleFB
        turnAngle(serPort, 0.2, angleToTurn);
        
    elseif smallestDist(1) == 1 &&  smallestDist(2) == 3
        % wall is front and left - must turn clockwise
        [angleFB angleRL wallLength] = triangWall(sonarArray(1), sonarArray(3));
        % turn CW
        angleToTurn = 180-angleFB
        turnAngle(serPort, 0.2, convertAngles(angleToTurn));
        
    elseif smallestDist(1) == 4 &&  smallestDist(2) == 2
        % wall is rear and right
        [angleFB angleRL wallLength] = triangWall(sonarArray(4), sonarArray(2));
        angleToTurn = 180-angleFB
        turnAngle(serPort, 0.2, convertAngles(angleToTurn));
        
    elseif smallestDist(1) == 4 &&  smallestDist(2) == 3
        % wall is rear and left
        [angleFB angleRL wallLength] = triangWall(sonarArray(4), sonarArray(2));
        angleToTurn = 180-angleFB
        turnAngle(serPort, 0.2, convertAngles(angleToTurn));

    end
    SetFwdVelAngVelCreate(serPort,0.2,0)
end


function angle = convertAngles(angle)
        angle = (-1).*angle
end


function botConfused(serPort)
sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
if sonarArray(1) < 2
   while sonarArray(1)<2 
    SetFwdVelAngVelCreate(serPort,0.5,0)
    sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
   end
elseif sonarArray(4) < 2
   while sonarArray(4)<2 
    SetFwdVelAngVelCreate(serPort,-0.5,0)
    sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
   end
end
    
end

function [angFB angLR wallLength] = triangWall(sensorFB, sensorLR)
%triangWall uses two sonar senors, which are placed 90deg apart, to deduce
%the angle of the bot in relation to the wall 
    wallLength = sqrt(sensorFB.^2 + sensorLR.^2);
    angFB = asind(sensorFB/wallLength)
    angLR = asind(sensorLR/wallLength)
end

function returnAngle = decideWhichAngle(sonarArray)
    smallestDist = find(sonarArray == min(sonarArray));
    hotSensors = find(sonarArray < 3);
    counter = length(hotSensors);
    for ii=1:counter
        hotSensors(ii) = sonarArray(hotSensors(ii));
    end
    %returnAngle = hotSensors;
    [triangulated(1) triangulated(2) triangulated(3)] = triangWall(hotSensors(1), hotSensors(2));
    returnAngle = [triangulated(2) triangulated(3)];

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