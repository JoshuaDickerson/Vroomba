function finalRad= ControlProgram(serPort)

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
    while toc(tStart) < maxDuration
        
        %angTurned= angTurned+AngleSensorRoomba(serPort);
        
        [sonarHot stopToken] = sonarCheckReact(serPort, stopToken);
    
%  Below is the very beginning of an avoidance system
            
%             if sonarArray(1) < 0.3
%                 sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
%                 SetFwdVelAngVelCreate(serPort,0,0)
%                 pause(0.1);
%                 smallestDist = find(sonarArray == min(sonarArray(2), sonarArray(3)));
%                 %SetFwdVelAngVelCreate(serPort,-0.3,0)
%                 pause(0.2)
%                 SetFwdVelAngVelCreate(serPort,0.0,0)
%                 sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
%                 if sonarArray(1) < 1.3
%                     if smallestDist == 3
%                         angle=convertAngles(3, 'cw');
%                         turnAngle(serPort, 0.2, angle);
%                         sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
%                     else
%                         angle=3;
%                         turnAngle(serPort, 0.2, angle);
%                         sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
%                     end
%                 else
%                 % Briefly pause to avoid continuous loop iteration
%                 pause(0.1)
%                 SetFwdVelAngVelCreate(serPort,0.05,0)
%                 end
%             end
        
    end
    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w)

end


function [sonarHot stopToken] = sonarCheckReact(serPort, stopToken)
% sonarCheckReact takes two arguments, serPort and a stopToken and performs
% baseline response to a wall detection
    sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
    % iterateCount is for debugging purposes, to allow us to see the
    % program looping
    global iterateCount
    
    % smallestDist(1) = index of the shortest sonar beams Right or Left
    % smallestDist(2) = index of the shortest sonar beams Front or Rear 
    smallestDist(1) = find(sonarArray == min(sonarArray(2), sonarArray(3)))
    smallestDist(2) = find(sonarArray == min(sonarArray(1), sonarArray(4)))
    
    
    if any(sonarArray) < 0.2
       stopBot(serPort)
       sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
       % smallestDist(1) = index of the shortest sonar beams Front or Rear
       % smallestDist(2) = index of the shortest sonar beams Right or Left 
       smallestDist(2) = find(sonarArray == min(sonarArray(2), sonarArray(3)))
       smallestDist(1) = find(sonarArray == min(sonarArray(1), sonarArray(4)))
       reactToWall(serPort, smallestDist);

       
       smallestDist = find(sonarArray == min(sonarArray(2), sonarArray(3)));
       [ang1 ang2 wallLength] = triangWall(sonarArray(1), sonarArray(smallestDist));
       if ang1/ang2 < 2 && sonarArray(2)<1 && sonarArray(3)<1
           disp('in a corner')
       end
       while smallestDist == 1
        % while the front sonar is smallest, do the following
        turnAngle(serPort, 0.2, 3);
        sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
        smallestDist = find(sonarArray == min(sonarArray(2), sonarArray(3)));
       end
        sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
        smallestDistSide = find(sonarArray == min(sonarArray(2), sonarArray(3)))
        smallestDistFR = find(sonarArray == min(sonarArray(1), sonarArray(4)))
       [ang1 ang2 wallLength] = triangWall(sonarArray(smallestDistFR), sonarArray(smallestDistSide));
        
       
             
       if smallestDist == 2 && sonarArray(1)<2
           angToTurn = ang2;
       elseif smallestDist == 3 && sonarArray(1)<2
           angToTurn = -1.*ang2;
       elseif smallestDist == 3 && sonarArray(4)<2
           angToTurn = -1.*ang2;
       elseif smallestDist == 2 && sonarArray(4)<2
           angToTurn = ang2;
       else angToTurn = 45;
       end
           if stopToken == 0
               turnAngle(serPort, 0.2, angToTurn);
               pause(0.1)
               stopToken = 1;
           else 
               pause(0.1)
               SetFwdVelAngVelCreate(serPort,0.2,0)
               stopToken = 0;
           end
           sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
       
    end
    pause(0.1)
    sonarHot = min(sonarArray);
    iterateCount = iterateCount+1;    
end

function stopBot(serPort)
SetFwdVelAngVelCreate(serPort,0,0)
distTraveled = DistanceSensorRoomba(serPort)
end


function reactToWall(serPort, smallestDist)
% reactToWall takes 2 arguments serPort, and a vector containing 2 integers
% smallestDist(1) = the index of the shortest sonar beam , front or rear
% (1, 4)
% smallestDist(2) = the index of the shortest sonar beam , Left or Right
% (3, 2)
    sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
    if sonarArray(2)>1 && sonarArray(3)>1 && sonarArray(4)>1
        % wall is just front
        turnAngle(serPort, 0.2, 90)
    elseif sonarArray(2)>1 && sonarArray(3)>1 && sonarArray(1)>1
        %wall is just rear
        turnAngle(serPort, 0.2, 90)
    elseif smallestDist(1) == 1 &&  smallestDist(2) == 2
        % wall is front and right
        turnAngle(serPort, 0.2, angleToTurn);
    elseif smallestDist(1) == 1 &&  smallestDist(2) == 3
        % wall is front and left
        turnAngle(serPort, 0.2, convertAngles(angleToTurn));
    elseif smallestDist(1) == 4 &&  smallestDist(2) == 2
        % wall is rear and right
    elseif smallestDist(1) == 4 &&  smallestDist(2) == 3
        % wall is rear and left
    end
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

function [angB angC wallLength] = triangWall(sensorC, sensorB)
%triangWall uses two sonar senors, which are placed 90deg apart, to deduce
%the angle of the bot in relation to the wall 
    wallLength = sqrt(sensorC.^2 + sensorB.^2);
    angB = asind(sensorC/wallLength)
    angC = asind(sensorB/wallLength)
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