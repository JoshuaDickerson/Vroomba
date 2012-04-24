function finalrad= examplecontrolprogram(serport)
% simple program for autonomously control the irobot create on either the
% physical create or the simulated version. this will simply spiral outward
% and turn away from obstacles that detects with the bump sensors.
%
% for the physical create only, it is assumed that the function call
% serport= roombainit(comport) was done prior to running this program.
% calling roombainit is unnecessary if using the simulator.
%
% input:
% serport - serial port object, used for communicating over bluetooth
%
% output:
% finalrad - double, final turning radius of the create (m)

% examplecontrolprogram.m
% copyright (c) 2011 cornell university
% this code is released under the open-source bsd license.  a copy of this
% license should be provided with the software.  if not, email:
% creatematlabsim@gmail.com

% set constants for this program
maxduration= 9999;  % max time to allow the program to run (s)
maxdistsansbump= 999; % max distance to travel without obstacles (m)
maxfwdvel= 0.5;     % max allowable forward velocity with no angular
% velocity at the time (m/s)
maxvelincr= 0.005;  % max incrementation of forward velocity (m/s)
maxodomang= pi/4;   % max angle to move around a circle before
% increasing the turning radius (rad)


% initialize loop variables
tstart= tic;        % time limit marker
distsansbump= 0;    % distance traveled without hitting obstacles (m)
angturned= 0;       % angle turned since turning radius increase (rad)
v= 0.3;               % forward velocity (m/s)
w= 0.0;          % angular velocity (rad/s)

% start robot moving
setfwdvelangvelcreate(serport,v,w)

% enter main loop
while toc(tstart) < maxduration && distsansbump <= maxdistsansbump
    distsansbump= distsansbump+distancesensorroomba(serport);
    angturned= angturned+anglesensorroomba(serport);
    
    % below if-loop is for moving the bot for a specific amount of time
    %         if toc(tstart) >= 8
    %             w = 0.4;
    %             setfwdvelangvelcreate(serport,v,w)
    %             if toc(tstart)>=12
    %                 w = -0.4;
    %                 setfwdvelangvelcreate(serport,v,w)
    %             end
    %         end
    %
    
    
    
    %  below is the very beginning of an avoidance system
    %  only works when the bot is running into a wall, and is at an angle
    %  to the wall, such that the right beam can see the wall
    sonararray = [readsonar(serport, 2) readsonar(serport, 1) readsonar(serport, 3) readsonar(serport,4 )];
    if any(sonararray<= 0.2)
        % smallestdist is the sensor with the shortest distance
        smallestdist = find(sonararray == min(sonararray));
        if smallestdist == 1
            setfwdvelangvelcreate(serport,0,0)
            turnangle(serport, 0.2, 45)
        elseif smallestdist == 4
            setfwdvelangvelcreate(serport,0,0.3)
        elseif smallestdist == 3
            turnparallelwall(serport, sonararray(1), -1.*sonararray(3))
        else
            turnparallelwall(serport, sonararray(1), sonararray(2))
        end
    end
    % briefly pause to avoid continuous loop iteration
    pause(0.1)
end

% specify output parameter
finalrad= v/w;

% stop robot motion
v= 0;
w= 0;
setfwdvelangvelcreate(serport,v,w)

%     % if you call roombainit inside the control program, this would be a
%     % good place to clean up the serial port with...
%     % fclose(serport)
%     % delete(serport)
%     % clear(serport)
%     % don't use these if you call roombainit prior to the control program
end

function turnparallelwall(serport, sonarfront, sonarright)
[angwall1 angwall2 walllength] = triangwall(sonarfront, sonarright);
pause(0.2);
turnangle(serport, 0.2, angwall2)
pause(0.1);
setfwdvelangvelcreate(serport,0.4,0)
end

function [angb angc walllength] = triangwall(sensorc, sensorb)
%triangwall uses two sonar senors, which are placed 90deg apart, to deduce
%the angle of the bot in relation to the wall
walllength = sqrt(sensorc.^2 + sensorb.^2);
angb = asind(sensorc/walllength);
angc = asind(sensorb/walllength);
end


function w= v2w(v)
% calculate the maximum allowable angular velocity from the linear velocity
%
% input:
% v - forward velocity of create (m/s)
%
% output:
% w - angular velocity of create (rad/s)

% robot constants
maxwheelvel= 2.5;   % max linear velocity of each drive wheel (m/s)
robotradius= 0.6;   % radius of the robot (m)

% max velocity combinations obey rule v+wr <= v_max
w= (maxwheelvel-v)/robotradius;
end