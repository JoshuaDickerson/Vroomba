%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Copyright 2010 Randolph Voorhies
%  This program is free software: you can redistribute it and/or modify
%  it under the terms of the GNU General Public License as published by
%  the Free Software Foundation, either version 3 of the License, or
%  (at your option) any later version.
%
%  This program is distributed in the hope that it will be useful,
%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  GNU General Public License for more details.
%
%  You should have received a copy of the GNU General Public License
%  along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The number of timesteps for the simulation
timesteps = 200;

% The maximum distance from which our sensor can sense a landmark
max_read_distance = 1.5;

% The actual positions of the landmarks (each column is a separate landmark)
real_landmarks = [1.0,  2.0,  0.0, 0.0, 1.0;     % x
    3.0,  2.5   3.4, 1.5, 3.5;     % y
    0.0,  0.0   0.0, 0.0, 0.0];    % Nothing

% The initial starting position of the robot
real_position = [0.0;      % x
    -1.0;     % y
    pi/3.0];  % rotation

% The movement command given tot he robot at each timestep
movement_command = [.05;     % Distance
    .01];    % Rotation

% The Gaussian variance of the movement commands
movement_variance = [.1;   % Distance
    .05]; % Rotation
M = [movement_variance(1), 0.0;
    0.0, movement_variance(2)];

% The Gaussian variance of our sensor readings
measurement_variance = [0.1;    % Distance
    0.01;   % Angle
    .0001]; % Landmark Identity
R = [measurement_variance(1), 0.0, 0.0;
    0.0, measurement_variance(2), 0.0;
    0.0, 0.0, measurement_variance(3)];

% Create the particles and initialize them all to be in the same initial
% position.
particles = [];
num_particles = 100;
for i = 1:num_particles
    particles(i).w = 1.0/num_particles;
    particles(i).position = real_position;
    for lIdx=1:size(real_landmarks,2)
        particles(i).landmarks(lIdx).seen = false;
    end
end

pos_history = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for timestep = 1:timesteps
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Move the actual robot
    real_position = moveParticle(real_position, movement_command, movement_variance);
    pos_history = [pos_history, real_position];
    
    % Move the actual particles
    for pIdx = 1:num_particles
        particles(pIdx).position = moveParticle( ...
            particles(pIdx).position, movement_command, movement_variance);
        particles(pIdx).position(3) = real_position(3);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Try to take a reading from each landmark
    doResample = false;
    for lIdx = 1:size(real_landmarks,2)
        real_landmark = real_landmarks(:, lIdx);
        
        % Take a real (noisy) measurement from the robot to the landmark
        [z_real, G] = getMeasurement(real_position, real_landmark, measurement_variance);
        read_distance(lIdx) = z_real(1);
        read_angle(lIdx)    = z_real(2);
        
        % If the landmark is close enough, then we can spot it
        if(read_distance(lIdx) < max_read_distance)
            doResample = true;
            
            for pIdx = 1:num_particles
                
                if(particles(pIdx).landmarks(lIdx).seen == false)
                    
                    % If we have never seen this landmark, then we need to initialize it.
                    % We'll just use whatever first reading we recieved.
                    particles(pIdx).landmarks(lIdx).pos = [particles(pIdx).position(1) + cos(read_angle(lIdx))*read_distance(lIdx);
                        particles(pIdx).position(2) + sin(read_angle(lIdx))*read_distance(lIdx);
                        0];
                    % Initialize the landmark position covariance
                    particles(pIdx).landmarks(lIdx).E = inv(G) * R * inv(G)';
                    
                    particles(pIdx).landmarks(lIdx).seen = true;
                    
                else
                    % Get an ideal reading to our believed landmark position (note 0 variance here).
                    [z_p, Gp] = getMeasurement(particles(pIdx).position, particles(pIdx).landmarks(lIdx).pos, [0;0]);
                    residual = z_real - z_p;
                    
                    %Calculate the Kalman gain
                    Q = G' * particles(pIdx).landmarks(lIdx).E * G + R;
                    K = particles(pIdx).landmarks(lIdx).E * G * inv(Q);
                    
                    % Mix the ideal reading, and our actual reading using the Kalman gain, and use the result
                    % to predict a new landmark position
                    particles(pIdx).landmarks(lIdx).pos = particles(pIdx).landmarks(lIdx).pos + K*(residual);
                    
                    % Update the covariance of this landmark
                    particles(pIdx).landmarks(lIdx).E = (eye(size(K)) - K*G')*particles(pIdx).landmarks(lIdx).E;
                    
                    % Update the weight of the particle
                    particles(pIdx).w = particles(pIdx).w * norm(2*pi*Q).^(-1/2)*exp(-1/2*(residual)'*inv(Q)*(residual));
                end %else
            end %pIdx
        end %distance
        
        
    end %for landmark
    % Resample all particles based on their weights
    if(doResample)
        particles = resample(particles);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % PLOTTING
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clf;
    hold on;
    
    % Plot the landmarks
    for lIdx=1:size(real_landmarks,2)
        plot(real_landmarks(1,lIdx), real_landmarks(2,lIdx), 'b*');
    end
    
    for lIdx = 1:size(real_landmarks,2)
        if(particles(1).landmarks(lIdx).seen)
            avg_landmark_guess =[0;0;0];
            for pIdx = 1:length(particles)
                avg_landmark_guess = avg_landmark_guess + particles(pIdx).landmarks(lIdx).pos;
            end
            avg_landmark_guess = avg_landmark_guess / length(particles);
            plot(avg_landmark_guess(1), avg_landmark_guess(2), 'ko');
        end
    end
    
    % Plot the particles
    particles_pos = [particles.position];
    plot(particles_pos(1,:), particles_pos(2,:), 'r.');
    
    % Plot the real robot
    plot(pos_history(1,:), pos_history(2,:), 'r');
    w = .1;
    l = .3;
    x = real_position(1);
    y = real_position(2);
    t = real_position(3);
    plot(real_position(1), real_position(2), 'mo', ...
        'LineWidth',1.5, ...
        'MarkerEdgeColor','k', ...
        'MarkerFaceColor',[0 1 0], ...
        'MarkerSize',10);
    
    % Show the sensor measurement as an arrow
    for lIdx=1:size(real_landmarks,2)
        real_landmark = real_landmarks(:, lIdx);
        if(read_distance(lIdx) < max_read_distance)
            line([real_position(1), real_position(1)+cos(read_angle(lIdx))*read_distance(lIdx)], ...
                [real_position(2), real_position(2)+sin(read_angle(lIdx))*read_distance(lIdx)]);
        end
    end
    
    axis([-5, 5, -4, 7]);
    pause(.01);
end

function newpos = updateMovement(pos, movement, variance)
% Compute how the robot should move from "pos" given the requested movement and
% some Gaussian random noise using a very simple motion model. This method is
% used to move the simulated robot as well as each of the hypothetical
% particles.

% Add some Gaussian random noise to the movement. Note that we are using
% a smaller variance in this Gaussian distribution, as the algorithm seems
% to work better when it underestimates the quality of the robot plant.
speed    = normrnd(movement(1), variance(1)*.25);
rotation = normrnd(movement(2), variance(2)*.25);

delta = zeros(3,1);
delta(1,1) = cos(pos(3)+rotation)*speed;
delta(2,1) = sin(pos(3)+rotation)*speed;
delta(3,1) = rotation;

newpos = pos+delta;
end

function [newParticles] = resample(oldParticles)
% Resample a group of particles such that those with higher weights have a
% higher chance of being replicated, and those with low weights have a high
% chance of disappearing
  
  weightSum = 0;
  for i=1:length(oldParticles)
    weightSum = weightSum + oldParticles(i).w;
  end
  for i=1:length(oldParticles)
    oldParticles(i).w = oldParticles(i).w/weightSum;
  end

  M = length(oldParticles);

  newParticles = [];

  r = rand / M;

  c = oldParticles(1).w;

  i = 1;

  for m=1:M
    U = r + (m-1) * M^(-1);

    while U > c
      i = i+1;
      c = c+oldParticles(i).w;
    end
    newParticles = [newParticles, oldParticles(i)];
  end

  for i=1:length(newParticles)
    newParticles(i).w = 1.0/length(newParticles);
  end

end

function [z, H] = getMeasurement(pos, landmark_pos, observation_variance)
%   Given a landmark position and a robot position, this method will return a
%   "measurement" z that contains the distance and the angle to the landmark.
%   Gaussian random noise is added to both based on the variances given in the
%   diagonal of the observation_variance matrix. Note that this method is used
%   both to take a "real" measurement in the simulation, as well as to assess 
%   what kind of measurement each of our hypothetical particles would take.
%   This method also computes the Jacobian of the measurent function for use
%   in an extended Kalman filter.


  % Compute the distance from the current position to the landmark, and add
  % some Gaussian noise to make things interesting. Note that we are using
  % a smaller variance in this Gaussian distribution, as the algorithm seems
  % to work better when it underestimates the quality of the sensor. 
  vector_to_landmark = [landmark_pos(1) - pos(1); landmark_pos(2) - pos(2)];
  landmark_distance = norm(vector_to_landmark);
  landmark_distance = landmark_distance + normrnd(0, observation_variance(1)*.25);

  % Compute the angle from the given pos to the landmark
  landmark_angle = atan2(vector_to_landmark(2), vector_to_landmark(1));
  landmark_angle = landmark_angle + normrnd(0, observation_variance(2)*.25);


  % Compute the Jacobian of this measurement function
  q = landmark_distance^2.0;
  H = [-(landmark_pos(1) - pos(1))/sqrt(q), -(landmark_pos(2) - pos(2))/sqrt(q), 0.0;
        (landmark_pos(2) - pos(2))/q,       -(landmark_pos(1) - pos(1))/q,      -1.0;
        0.0,                                0.0,                               1.0];

  z = [landmark_distance; 
       landmark_angle;
       0];
end

