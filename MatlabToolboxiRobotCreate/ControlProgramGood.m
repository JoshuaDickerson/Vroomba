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
    
    % define tolerances (how far from wall bot needs to react)
    frTolerance = 0.5;
    lrTolerance = 0.3;
    % Start robot moving
    stopToken = 0;
    % Enter main loop 
    while toc(tStart) < maxDuration.*1000;
        if stopToken == 0
            SetFwdVelAngVelCreate(serPort,0.3,0); 
        else
           SetFwdVelAngVelCreate(serPort,0.0,0);
        end
         sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];        
        
         if any(sonarArray <lrTolerance)
             stopBot(serPort);
             reactToWall(serPort, sonarArray);
         elseif sonarArray(1) <= frTolerance || sonarArray(2) <=lrTolerance || sonarArray(3)<=lrTolerance
           stopBot(serPort);
           sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
           reactToWall(serPort, sonarArray);
        end
        
%         sonarCheck(serPort);
        pause(0.1)
    end
pause(0.1)

end



function reactToWall(serPort, sonarArray)
disp('reactToWall envoked')
    % reactToWall takes 2 arguments serPort, and a vector containing all sonar
    % readings
    global stopToken;
    global zeroDistCount;
    smallestDist = [3 3 3 3];
    % stopToken = 1; prevents bot from moving
    stopToken = 1;
    stopBot(serPort)
    
    % zeroDistCount iterates every time the turnBot() function ius called
    % and the distance traveled is 0m
    if zeroDistCount == 3
        botConfused(serPort);
    end
    
   
        % smallestDist(1) = index of the shortest sonar beams Front or Rear
        % smallestDist(2) = index of the shortest sonar beams Right or Left
        if sonarArray(2) == 3.00 && sonarArray(3) == 3.00
            % neither left or right beams making contact
            % robot perpendicular to a wall
        else 
            smallestDist(2) = find(sonarArray == min(sonarArray(2), sonarArray(3)));
        end



        try
            if sonarArray(1) == 3.00 && sonarArray(4) == 3.00
                if any(sonarArray <0.3)
                   smallestDist(1) = find(sonarArray == min(sonarArray));
                   if smallestDist(1) == 2
                       turnBot(serPort, 45)
                   elseif smallestDist(1) == 3
                       turnBot(serPort, -45)
                   end
                else
                    % neither front or rear beams making contact
                    % Bot parallel to a wall
                    % just keep walking
                    stopToken = 0;
                    smallestDist(1) = find(sonarArray == min(sonarArray));
                end
            else
                smallestDist(1) = find(sonarArray == min(sonarArray(1), sonarArray(4)));

                if sonarArray(2)>0.6 && sonarArray(3)>0.6 && sonarArray(4)>0.6
                    % wall is just front
                    disp('Wall is only front')
                    turnBot(serPort, 90);
                    stopToken = 0;
                elseif sonarArray(2)>0.6 && sonarArray(3)>0.6 && sonarArray(1)>0.6
                    %wall is just rear
                    disp('Wall is only rear')
                    %turnBot(serPort, convertAngles(90))
                    stopToken = 0;
                elseif smallestDist(1) == 1 &&  smallestDist(2) == 2
                    % wall is front and right -- must turn ccw
                    [angleFB angleRL wallLength] = triangWall(sonarArray(1), sonarArray(2));
                    angleToTurn = 90-angleFB;
                    fprintf('wall is front/right -- must turn ccw %0.1f\n', angleToTurn)
                    turnBot(serPort, angleToTurn)
                    stopToken = 0;
                elseif smallestDist(1) == 1 &&  smallestDist(2) == 3
                    % wall is front and left - must turn clockwise
                    [angleFB angleRL wallLength] = triangWall(sonarArray(1), sonarArray(3));
                    % turn CW
                    angleToTurn = angleFB;
                    fprintf(' wall is front/left -- must turn cw %0.1f\n', angleToTurn)
                    turnBot(serPort, convertAngles(angleToTurn))
                    stopToken = 0;
                elseif smallestDist(1) == 4 &&  smallestDist(2) == 2
                    disp('Wall is right and rear');
                   if sonarArray(2)<0.05
                      turnBot(serPort, 45)
                      stopToken = 0;
                      walkFwd(serPort, 0.1);
                      SetFwdVelAngVelCreate(serPort,0.3,0);
                   else
                       % wall is rear and right
                       %[angleFB angleRL wallLength] = triangWall(sonarArray(4), sonarArray(2));
                       %angleToTurn = 180-angleFB;
                       %turnBot(serPort, angleToTurn);
                       stopToken = 0;
                       walkFwd(serPort, 0.1);
                       SetFwdVelAngVelCreate(serPort,0.3,0);
                   end
                elseif smallestDist(1) == 4 &&  smallestDist(2) == 3
                    disp('Wall is left and rear');
                   if sonarArray(2)<0.05
                      turnBot(serPort, -45)
                      stopToken = 0;
                      walkFwd(serPort, 0.1);
                      SetFwdVelAngVelCreate(serPort,0.3,0);
                   else
                       % wall is rear and right
                       %[angleFB angleRL wallLength] = triangWall(sonarArray(4), sonarArray(2));
                       %angleToTurn = 180-angleFB;
                       %turnBot(serPort, angleToTurn);
                       stopToken = 0;
                       walkFwd(serPort, 0.1);
                       SetFwdVelAngVelCreate(serPort,0.3,0);
                   end
                end
            end
            SetFwdVelAngVelCreate(serPort,0.3,0);
            disp('reactToWall COMPLETE')
    
        catch err
            disp(err);
        end
    
end

    
function walkFwd(serPort, distance)
disp('walkFwd envoked')
global stopToken;
    if stopToken == 0;
        travelDist(serPort, 0.2, distance);
    end
disp('walkFwd COMPLETE')  
end


function turnBot(serPort, angleToTurn)
    disp('turnBot envoked')
    global tStart;
    global zeroDistCount;
    distTraveled = DistanceSensorRoomba(serPort);
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
    disp('turnBot COMPLETE')
end

function angle = convertAngles(angle)
        angle = (-1).*angle;
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
     disp('botConfused -- COMPLETE')
 end

function [angFB angLR wallLength] = triangWall(sensorFB, sensorLR)
%triangWall uses two sonar senors, which are placed 90deg apart, to deduce
%the angle of the bot in relation to the wall 
    wallLength = sqrt(sensorFB.^2 + sensorLR.^2);
    angFB = asind(sensorFB/wallLength);
    angLR = asind(sensorLR/wallLength);
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
    disp('stopBot envoked')
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