%% Skidpad Event XY Coordinates
% Columbia FSAE

% Create initial between points for straight line
interval = 2*pi/360;

% Length of the straight for the start of the event
% NOTE: timing starts after the straight section is complete
straight_start = (21.25/2) + 3; %3 meters is an estimation

% Create start point for array (will be flipped later so we take max
% length)
y(1,1) = straight_start;
j = 2; % counter

% Loop to continually create data points increasing by interval
while (straight_start - j*interval) >= 0
    
    y(j,1) = straight_start - j*interval;
    
    j = j + 1;
    
end 

% Add start point or origin
y(j,1) = 0;

% Flip the array I build (recall it started at the end)
y = flipud(y);
% We only move in y direction, so make a vector of zeroes
x = zeros(length(y),1);
m = [x, y];

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

% Shift circles
right_laps(:,2) = right_laps(:,2) + m(length(m),2);
left_laps(:,2) = left_laps(:,2) + m(length(m),2);

% This is the result that should be the input for the actual sim data
skidpad = [m; right_laps; left_laps];

% Use plot to check results
%hold on
%axis equal
%plot(skidpad(:,1), skidpad(:,2))
%% Car specs + Defining Sector
global poweroutput; % power output of the car's engine
global weight; % weight of the car
global mass; % mass of the car
global tirefriction; % coefficent of friction of the tires
global Af; % frontal area of the car
global Cd; % drag coefficient
global airdensity; % density of air
global R; % normal force on tire
global g; % acceleration due to gravity

% With Wings
global Aw; % wing area
global Cwd; % wing coefficient of drag
global Cl; % wing coefficient of lift

% Test Parameter Values
poweroutput = 67.453411; % (bhp)
weight = 2893.95;
mass = 295; % 227kg +68kg driver
tirefriction = 1;
Af = 1.21;
Cd = 0.74;
airdensity  = 1.162;
g = 9.81;
R = 1;
Aw = 1;
Cwd = 1;
Cl = 1;

% Every 20 points constitutes a sector
sectorIndex = 0:20:length(skidpad);
sectorIndex(1) = 1;
sectorIndex;
%% Corner Radius
% Creates an array of corner radius using sector lengths
CornerRadiusArray = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    if i == 1
        CornerRadiusArray(i) = 0;
    elseif i > length(sectorIndex) - 1
        CornerRadiusArray(i) = 0;
    else
        a = sectorLength(skidpad(sectorIndex(i-1),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i-1),2),skidpad(sectorIndex(i+1),2)); % length from next sector to previous
        b = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2)); % current sector to next
        c = sectorLength(skidpad(sectorIndex(i-1),1),skidpad(sectorIndex(i),1),skidpad(sectorIndex(i-1),2),skidpad(sectorIndex(i),2)); % previous to current

        CornerRadiusArray(i) = CornerRadius(a,b,c);
    end
end

format shortG
CornerRadiusArray % Array of corner radius at each sector!
% Straight sector radius = infinity
%% Maximum Corner Velocity
% Creates an array of maximum corner velocity using corner radius, assuming
% no downforce
MaxCornerSpeedArray = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    MaxCornerSpeedArray(i) = MaximumCornerSpeed(CornerRadiusArray(i));
end
MaxCornerSpeedArray % Array of maximum corner speed at each sector!
%% Top Speed for Straight Sector
% Creates an array of maximum velocity for straight sectors using
% s (distance travelled) and u (entry speed)
MaxSpeedArray = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    if i > length(sectorIndex)+1
        slength = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2));
    else
        slength = 0;
    end
    MaxSpeedArray(i) = EndofSectorSpeed(slength,MaxCornerSpeedArray(i)); 
    % we take the corner speed as entry speed for now...
end
MaxSpeedArray % Array of maximum speed at each straight sector!
%% Functions
function [a] = sectorLength(x1,x2,y1,y2)
    a = ((x2 - x1)^2 + (y2 - y1)^2)^(1/2);
end

function [radius] = CornerRadius (a,b,c)
    A = acosd((b^2 + c^2 - a^2)/(2*b*c));
    radius = a/(2*sind(180 - A));
end

function [maxcornerspeed] = MaximumCornerSpeed(radius)
    global R;
    global tirefriction;
    global mass;
    global airdensity;
    global Af;
    global Cd;    
    maxcornerspeed = ((tirefriction*R)^2/((mass/radius)^2 + (0.5*airdensity*Af*Cd)^2))^(1/4);
end

function [endofsectorspeed] = EndofSectorSpeed (s,u)
    global poweroutput;
    global airdensity;
    global Af;
    global mass;
    global Cd;
    endofsectorspeed = (u^2 + s*2*((poweroutput/u)-0.5*airdensity*u^2*Af*Cd)/mass)^(1/2);
end
