function finalRad= ControlProgram(serPort)

    % git@github.com:/JoshuaDickerson/Vroomba.git


    % Set constants for this program
    maxDuration= 999;  % Max time to allow the program to run (s)
    maxDistSansBump= 999; % Max distance to travel without obstacles (m)
    maxFwdVel= 0.5;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
    maxVelIncr= 0.005;  % Max incrementation of forward velocity (m/s)
    maxOdomAng= pi/4;   % Max angle to move around a circle before 
                        % increasing the turning radius (rad)
    
    
    % Initialize loop variables
    global tStart;
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    angTurned= 0;       % Angle turned since turning radius increase (rad)
    v= 0.0;               % Forward velocity (m/s)
    w= 0.0;          % Angular velocity (rad/s)
    global iterateCount;
    global stopToken;
    global zeroDistCount;
    zeroDistCount = 0;
    iterateCount = 0;
    
    lrTolerance = 0.2
    fbTolerance = 0.3
    % Start robot moving
    stopToken = 0;
    % Enter main loop 
    while toc(tStart) < maxDuration.*1000;
        if stopToken == 0
            SetFwdVelAngVelCreate(serPort,0.3,0) 
        else
           SetFwdVelAngVelCreate(serPort,0.0,0)
        end
         sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
        %angTurned= angTurned+AngleSensorRoomba(serPort);
        %if length(find(sonarArray < 0.09))>=2
        % if multiple sonars are too close to a wall
        %stopBot(serPort);
        %pause(30);
        %end
%         maintainRat = ratioWalker(serPort);
%         if maintainRat == 1
%             disp('maintain rat');
%         end
        
        if sonarArray(1) <= fbTolerance || sonarArray(2) <=lrTolerance || sonarArray(3)<=lrTolerance
           stopBot(serPort)
           sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
           reactToWall(serPort, sonarArray);
        end
        
%         sonarCheck(serPort);
        pause(0.1)
    end
    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w)

end





% function sonarCheck(serPort)
% % sonarCheckReact takes two arguments, serPort and a stopToken and performs
% % baseline response to a wall detection
%     sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
%     % iterateCount is for debugging purposes, to allow us to see the
%     % program looping
%     global iterateCount;
%     global stopToken;
%     if any(sonarArray) < 0.08
%        sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
%        reactToWall(serPort, sonarArray);
%     end
%     stopToken = 0;
%     iterateCount = iterateCount+1;    
% end




function reactToWall(serPort, sonarArray)
    global stopToken;
    global zeroDistCount;
    stopToken = 1;
    stopBot(serPort)
    
    if zeroDistCount == 3
        botConfused(serPort);
    end
   
% reactToWall takes 2 arguments serPort, and a vector containing all sonar
% readings
    
    % smallestDist(1) = index of the shortest sonar beams Front or Rear
    % smallestDist(2) = index of the shortest sonar beams Right or Left
    if sonarArray(2) == 3.00 && sonarArray(3) == 3.00
        % neither left or right beams making contact
        % Bot perpendicular to a wall
    else 
        smallestDist = findSmallestDist(serPort, sonarArray);
    end
    
    if sonarArray(1) == 3.00 && sonarArray(4) == 3.00
        % neither front or rear beams making contact
        % Bot parallel to a wall
        % just keep walking
        stopToken = 0;
    else
        smallestDist = findSmallestDist(serPort, sonarArray);
        if sonarArray(2)>1.3 && sonarArray(3)>1.3 && sonarArray(4)>1
            % wall is just front
            turnBot(serPort, 90)
            stopToken = 0;
        elseif sonarArray(2)>1.3 && sonarArray(3)>1.3 && sonarArray(1)>1
            %wall is just rear
            turnBot(serPort, convertAngles(90))
            stopToken = 0;
        elseif smallestDist(1) == 1 &&  smallestDist(2) == 2
            % wall is front and right -- must turn ccw
            disp(' wall is front/right -- must turn ccw')
            [angleFB angleRL wallLength] = triangWall(sonarArray(1), sonarArray(2));
            angleToTurn = 90-angleFB
            turnBot(serPort, angleToTurn)
            stopToken = 0;
        elseif smallestDist(1) == 1 &&  smallestDist(2) == 3
            % wall is front and left - must turn clockwise
            disp(' wall is front/left -- must turn cw')
            [angleFB angleRL wallLength] = triangWall(sonarArray(1), sonarArray(3));
            % turn CW
            angleToTurn = angleFB
            turnBot(serPort, convertAngles(angleToTurn))
            stopToken = 0;
        elseif smallestDist(1) == 4 &&  smallestDist(2) == 2
           % wall is rear and right
           [angleFB angleRL wallLength] = triangWall(sonarArray(4), sonarArray(2));
           angleToTurn = 180-angleFB
           turnBot(serPort, angleToTurn);
           stopToken = 0;
        elseif smallestDist(1) == 4 &&  smallestDist(2) == 3
           % wall is rear and left
           %[angleFB angleRL wallLength] = triangWall(sonarArray(4), sonarArray(2));
           %angleToTurn = 180-angleFB
           %turnBot(serPort, convertAngles(angleToTurn));
           travelDist(serPort, 0.2, 0.20.*sonarArray(1));
           stopToken = 0;
        end
     end
    
end


function turnBot(serPort, angleToTurn)
    global tStart;
    global zeroDistCount;
    distTraveled = DistanceSensorRoomba(serPort)
    if distTraveled == 0
        zeroDistCount = zeroDistCount + 1;
    else
        zeroDistCount = 0;
    end
    turnAngle(serPort, 0.2, angleToTurn);
    fh = fopen('roombaLog.dat', 'a+');
    fprintf(fh, '%0.4f\t%0.4f\t%0.4f\n', toc(tStart), distTraveled, angleToTurn);
    fclose(fh);
    pause(0.1);
end

function angle = convertAngles(angle)
        angle = (-1).*angle
end

function [smallestDist] = findSmallestDist(serPort, sonarArray)
    try
        smallestDist(1) = find(sonarArray == min(sonarArray(1), sonarArray(4)))
        smallestDist(2) = find(sonarArray == min(sonarArray(3), sonarArray(2)))
    catch err
       disp(err);
       turnBot(serPort, 180);
       travelDist(serPort, 0.2, 0.20)
       stopBot(serPort);
       pause(0.1)
       sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];       
       smallestDist = findSmallestDist(serPort, sonarArray);
    end
end


 function botConfused(serPort)
 disp('botConfused -------------------')
 sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
 largestDist = find(sonarArray == max(sonarArray)) 
     if largestDist == 4
        turnBot(serPort, 180);
        travelDist(serPort, 0.2, sonarArray(4).*0.20)
     elseif largestDist == 3
        turnBot(serPort, 90);
        travelDist(serPort, 0.2, sonarArray(3).*0.20)
     elseif largestDist == 2
        turnBot(serPort, -90);
        travelDist(serPort, 0.2, sonarArray(2).*0.20)
     else
        travelDist(serPort, 0.2, sonarArray(1).*0.20)
     end
     
 end

function [angFB angLR wallLength] = triangWall(sensorFB, sensorLR)
%triangWall uses two sonar senors, which are placed 90deg apart, to deduce
%the angle of the bot in relation to the wall 
    wallLength = sqrt(sensorFB.^2 + sensorLR.^2);
    angFB = asind(sensorFB/wallLength)
    angLR = asind(sensorLR/wallLength)
end


function [maintainRat] = ratioWalker(serPort)
    sonarArrayInitial = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
    ratInitial = sonarArrayInitial(2)/sonarArrayInitial(1);
    % walk a step and check the ratio
    % if ratio is the same but one beam is smaller, we're at an angle to the
    % wall
    travelDist(serPort, 0.1, 0.1)
    pause(0.1)
    sonarArrayAfter = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
    ratAfter = sonarArrayAfter(2)/sonarArrayAfter(1);
    if abs(ratInitial - ratAfter)<=0.02
        if abs(sonarArrayInitial(2) - sonarArrayAfter(2)) >= 0.02 || abs(sonarArrayInitial(1) - sonarArrayAfter(1)) >= 0.02
            maintainRat = 1; % means we're at an angle, headed towards wall
        end
    else maintainRat = 0; 
    end
    
end

function stopBot(serPort)
    global stopToken;
    stopToken = 1;
    SetFwdVelAngVelCreate(serPort,0,0)
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