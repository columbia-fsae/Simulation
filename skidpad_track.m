
%% Skidpad Event XY Coordinates
% Columbia FSAE

% Create initial between points for circle
interval = 2*pi/360;

% Create circles to reflect rest of course

%2021 radius based off inner and outer diameters 
r=((15.25/2) + (21.25/2))/2; 
C=[1, 0];

% Make array of theta values to plug into equation
theta=0:interval:2*pi;

% Make laps 1 and 2
right_laps=r*[-cos(theta')+C(1) sin(theta')+C(2)];% the points you asked
right_laps = [right_laps(2:length(right_laps),:); right_laps(2:length(right_laps),:)];

% Make laps 3 and 4
left_laps=r*[cos(theta')-C(1) sin(theta')+C(2)];% the points you asked
left_laps = [left_laps(2:length(left_laps),:); left_laps(2:length(left_laps),:)];

% Length of the straight for the start of the event
% NOTE: timing starts after the straight section is complete
straight_start = (21.25/2) + 3; %3 meters is an estimation

% Create start point for array (will be flipped later so we take max
% length)
y(1,1) = straight_start;
j = 1; % counter

% Calculate total distance between points based on circle
distance = sqrt((right_laps(2,1) - right_laps(1,1))^2 + (right_laps(2,2) - right_laps(1,2))^2);

% Loop to continually create data points increasing by interval
while (straight_start - j*distance) >= 0
    
    y(j+1,1) = straight_start - j*distance;
    
    j = j + 1;
    
end 

% Add start point or origin
% DELETING FOR NOW

% Flip the array I build (recall it started at the end)
y = flipud(y);
% We only move in y direction, so make a vector of zeroes
x = zeros(length(y),1);
m = [x, y];

% Shift circles
right_laps(:,2) = right_laps(:,2) + m(length(m),2);
left_laps(:,2) = left_laps(:,2) + m(length(m),2);

% This is the result that should be the input for the actual sim data
skidpad = [m; right_laps; left_laps]

% Use plot to check results
hold on
axis equal
plot(skidpad(:,1), skidpad(:,2))


