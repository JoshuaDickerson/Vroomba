function finalRad= ControlProgram(serPort)

    % git@github.com:/JoshuaDickerson/Vroomba.git


    % Set constants for this program
    maxDuration= 999;  % Max time to allow the program to run (s)
    
    % Initialize config variables
    global tStart;
    tStart= tic;            % Time limit marker
    global iterateCount;    % counts the iteration of a function (debug only)
    global stopToken;       % token passed between functions, to prevent bot from moving
    global zeroDistCount;   % iterates if bot is turning but not moving fwd
    global debug;           % global debug variablw
    % debug=1 is in debug mode, not in debug == 0
    debug = 0;
    zeroDistCount = 0;
    iterateCount = 0;
    
    % define tolerances (how far from wall bot needs to react)
    frTolerance = 0.5;
    lrTolerance = 0.3;
    % Start robot moving
    stopToken = 0;
    
    % Enter main loop 
    while toc(tStart) < maxDuration.*1000;
        % stopToken prevents/allows movement 
        if stopToken == 0
            SetFwdVelAngVelCreate(serPort,0.3,0); 
        else
           SetFwdVelAngVelCreate(serPort,0.0,0);
        end
        
        % Bump sensors respond too slowly to be effective. This is a
        % condition for them anyways
        [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        Bumped = [BumpRight BumpLeft BumpFront];
        if any(Bumped)==1
            stopToken = 1;
            
            % debug line --------------------
            if debug == 1
                disp('BUMPED ENVOKED #######')
            end
           
            
            stopBot(serPort);
            walkBack(serPort);
            pause(0.1)
            stopBot(serPort)
            turnBot(serPort, 45)
            stopToken = 0;
        end

            
         % sonarArray actively gathers sonar data while bot moves   
         sonarArray = [ReadSonarMultiple(serPort, 2) ReadSonarMultiple(serPort, 1) ReadSonarMultiple(serPort, 3) ReadSonarMultiple(serPort,4 )];
         
         % if any sonar sensor reads below the LR tolerances set in the config
         % block, reactToWall() in envoked. This condition is used when the
         % front beam is long, but the side beams are short
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

    % reactToWall takes 2 arguments serPort, and a vector containing all sonar
    % readings
    global stopToken;
    global zeroDistCount;
    global debug;
    
    % debug line --------------------
    if debug==1
        disp('reactToWall envoked')
    end
   
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
            % if front and rear beams are not making contact
            if sonarArray(1) == 3.00 && sonarArray(4) == 3.00
                % if and beams are shorter than 0.3m, find the smallestDist
                if any(sonarArray <0.3)
                   smallestDist(1) = find(sonarArray == min(sonarArray));
                   % turn accordingly
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
                % find the shortest beam front/back
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
                       stopToken = 0;
                       walkFwd(serPort, 0.1);
                       SetFwdVelAngVelCreate(serPort,0.3,0);
                   end
                end
            end
            % if we didn't meet any of the above conditions, walk forward
            stopToken = 0;
            SetFwdVelAngVelCreate(serPort,0.3,0);
            
            % debug line ---------------------------------------
            if debug==1
                disp('reactToWall COMPLETE')
            end
        % if while loop catches any errors, envoke botFail();
        catch err
            disp(err);
            botFail(serPort);
        end
    
end

    
function walkFwd(serPort, distance)
% walkFwd takes 1 argument, and moves a specified distance before stopping
global debug;
global stopToken;

% debug line ------------------------------------  
if debug ==1
    disp('walkFwd envoked')
end

    if stopToken == 0;
        travelDist(serPort, 0.2, distance);
    end
    
% debug line ------------------------------------    
if debug ==1
    disp('walkFwd COMPLETE')
end
end

function walkBack(serPort, distance)
% same as walkFwd, only the distance moved is recorded as negative distance
% to protect the graph. This function is only envoked if bump sensors are
% triggered 
global debug;
global stopToken;
global tStart;
% debug line ------------------------------------
if debug ==1
    disp('walkBack envoked')
end

    if stopToken == 0;
        SetDriveWheelsCreate(serPort,-0.2,-0.2)
        pause(0.1)
        stopBot(serPort);
    end
    distTraveled = (-1).*DistanceSensorRoomba(serPort);
    angleToTurn = 0;
    fh = fopen('roombaLog.dat', 'a+');
    fprintf(fh, '%0.4f\t%0.4f\t%0.4f\n', toc(tStart), distTraveled, angleToTurn);
    fclose(fh);
    pause(0.1);
    
% debug line ------------------------------------    
if debug ==1
    disp('walkFwd COMPLETE')
end
end


function turnBot(serPort, angleToTurn)
% turnBot() takes an angle as an argument, and turns accordingly, then
% records travel data to the log file
    global tStart;
    global zeroDistCount;
    global debug;
    % debug line ------------------------------------
    if debug ==1 
        disp('turnBot envoked')
    end
    %distTraveled since last call of the turnBot() function
    distTraveled = DistanceSensorRoomba(serPort);
    
    % conditional increments global variable to indicate whether the bot
    % has moved since last turning, if bot has a distance=0 for three
    % iterations of turnBot(), confusedBot() is called
    if distTraveled == 0
        zeroDistCount = zeroDistCount + 1;
    else
        zeroDistCount = 0;
    end
    turnAngle(serPort, 0.2, angleToTurn);
    
    % record data to log file for creating tracks
    fh = fopen('roombaLog.dat', 'a+');
    fprintf(fh, '%0.4f\t%0.4f\t%0.4f\n', toc(tStart), distTraveled, angleToTurn);
    fclose(fh);
    pause(0.1);
    % debug line ------------------------------------
    if debug==1
        disp('turnBot COMPLETE')
    end
end

function angle = convertAngles(angle)
% convertAngles() converts from CCW to CW turning
        angle = (-1).*angle;
end


function botFail(serPort)
% botFail() is envoked as a result of the main try/catch statement. 
% if there is an error in the main loop, the bot turns 90 degrees and
% begins to walk forward
global debug;
if debug ==1 
    disp('botFail ENVOKED')
end

stopBot(serPort);
turnBot(serPort, 90);
SetFwdVelAngVelCreate(serPort,0.3,0);
end



 function botConfused(serPort)
 % botConfused() is envoked when the bot turns 3 times without changing
 % distance (bot is stuck)
 global debug;
 if debug ==1 
    disp('botConfused -------------------')
 end
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
     if debug==1
        disp('botConfused -- COMPLETE')
     end
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
% stopBot stops the robot, and returns a stopToken=1;
global debug;
global stopToken;
    % debug line ------------------------------------
    if debug ==1
        disp('stopBot envoked')
    end
    
    stopToken = 1;
    SetFwdVelAngVelCreate(serPort,0,0)
end
