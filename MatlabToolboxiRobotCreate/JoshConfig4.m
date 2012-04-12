function finalRad= ControlProgram(serPort)

    % Set constants for this program
    maxDuration= 9999;  % Max time to allow the program to run (s)
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
    
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w)
    
    % Enter main loop
    while toc(tStart) < maxDuration && distSansBump <= maxDistSansBump
        
        %angTurned= angTurned+AngleSensorRoomba(serPort);

        sonarHot = sonarCheckReact(serPort);
    
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


function sonarHot = sonarCheckReact(serPort)
    sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
    if sonarArray(1) < 0.2 || sonarArray(2)<0.1 || sonarArray(3)<0.1
       stopBot(serPort)
       smallestDist = find(sonarArray == min(sonarArray))
       while smallestDist == 1
        turnAngle(serPort, 0.2, 3)
        sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
        smallestDist = find(sonarArray == min(sonarArray))
       end
        sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
        smallestDist = find(sonarArray == min(sonarArray))
       [ang1 ang2 wallLength] = triangWall(sonarArray(1), sonarArray(smallestDist))
       
       if smallestDist == 2
           turnAngle(serPort, 0.2, ang2)
       elseif smallestDist == 3
           ang2 = -1.*ang2
           turnAngle(serPort, 0.2, ang2)
       end
       
    end
    pause(0.1)
    sonarHot = min(sonarArray);
    
end

function stopBot(serPort)
SetFwdVelAngVelCreate(serPort,0,0)
end



function angle = convertAngles(angle, direction)
    if direction =='cw'
        angle = (-1).*angle
    end
end



function [angB angC wallLength] = triangWall(sensorC, sensorB)
%triangWall uses two sonar senors, which are placed 90deg apart, to deduce
%the angle of the bot in relation to the wall 
    wallLength = sqrt(sensorC.^2 + sensorB.^2);
    angB = asind(sensorC/wallLength);
    angC = asind(sensorB/wallLength);
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