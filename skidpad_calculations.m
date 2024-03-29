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
skidpad = [m; right_laps; left_laps];

% Use plot to check results
% hold on
% axis equal
% plot(skidpad(:,1), skidpad(:,2))
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

% Test Parameter Values
poweroutput = 50300; % Watts
mass = 295; % 227kg+68kg driver
tirefriction = 1; % taken from LTS pdf
Af = 1.21;
Cd = 0.74;
airdensity  = 1.162;
g = 9.81;
R = mass*g;
weight = mass*g; % assuming 1 tire

% Every 'interval' point constitute a sector
interval = 3; % stop = 87, divisible by 3
sectorIndex = 0:interval:length(skidpad);
sectorIndex(1) = 1;

% Calculate index when timing starts, skidpad SPECIFIC!
for i = 1:length(skidpad)
    if skidpad(i,1) ~= 0
        index = i;
        break;
    end
end
stop = nearest(index/interval);
if stop == 0
    stop = 1;
end
%% Corner Radius
% Creates an array of corner radius using sector lengths
CornerRadiusArray = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    if i == 1
        CornerRadiusArray(i) = CornerRadiusArray(i+1);
    elseif i > length(sectorIndex) - 1
        CornerRadiusArray(i) = CornerRadiusArray(i-1);
    else
        a = sectorLength(skidpad(sectorIndex(i-1),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i-1),2),skidpad(sectorIndex(i+1),2)); % length from next sector to previous
        b = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2)); % current sector to next
        c = sectorLength(skidpad(sectorIndex(i-1),1),skidpad(sectorIndex(i),1),skidpad(sectorIndex(i-1),2),skidpad(sectorIndex(i),2)); % previous to current
        CornerRadiusArray(i) = real(CornerRadius(a,b,c)); % consider only the real part
    end
    
    if CornerRadiusArray(i) > 1000
     CornerRadiusArray(i) = CornerRadiusArray(i-1); % if radius is too big due to transitions, set it to previous radius
    end
    
end
format shortG
CornerRadiusArray; % Array of corner radius at each sector!
% Straight sector radius = infinity
%% Maximum Corner Velocity
% Creates an array of maximum corner velocity using corner radius, assuming
% no downforce
MaxCornerSpeed = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    MaxCornerSpeed(i) = MaximumCornerSpeed(CornerRadiusArray(i));
end
MaxCornerSpeed; % Array of maximum corner speed at each sector!
%% Entry and Exit Speed, Braking (page 16)
% Set up
ExitSpeed = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    ExitSpeed(i) = MaxCornerSpeed(i); % page 15 in LTS pdf
end
ExitSpeed; % unbraked

EntrySpeed = zeros(length(sectorIndex),1);
for i = 2:length(sectorIndex)
    EntrySpeed(1) = 0.001; % start from rest
    EntrySpeed(i) = ExitSpeed(i-1); % exit v of N = entry v for N+1
end
EntrySpeed; % unbraked

% First pass
for i = 1:length(sectorIndex)
    if EntrySpeed(i) > MaxCornerSpeed(i) % need to brake
        EntrySpeed(i) = MaxCornerSpeed(i);
    end
end

% Second pass
MaxEntry = zeros(length(sectorIndex),1);
for i = length(sectorIndex)-1:-1:1
    if EntrySpeed(i+1) < ExitSpeed(i)
        % calculate max entry (exit speed = equal to the (braked) entry speed for sector N+1
        s = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2));
        MaxEntry(i) = MaxEntrySpeed(DecelerativeForce(CornerRadiusArray(i),EntrySpeed(i+1)),EntrySpeed(i+1),s);
        if MaxEntry(i) > EntrySpeed(i) % transition to braking
            ExitSpeed(i) = EntrySpeed(i+1);
        else % maximum braking
            EntrySpeed(i) = MaxEntry(i);
            ExitSpeed(i) = EntrySpeed(i+1);
        end
    end
    % calculate max entry (original exit speed)
    s = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2));
    MaxEntry(i) = MaxEntrySpeed(DecelerativeForce(CornerRadiusArray(i),ExitSpeed(i)),ExitSpeed(i),s);
end
EntrySpeed;
ExitSpeed;
MaxEntry;
%% Top Speed for Straight Sector
% Creates an array of maximum velocity for straight sectors using
% s (distance travelled) and u (entry speed)
MaxStraightSpeed = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    if i > length(sectorIndex)+1
        slength = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2));
    else
        slength = 0;
    end
    MaxStraightSpeed(i) = EndofSectorSpeed(slength,EntrySpeed(i)); 
end
MaxStraightSpeed(isnan(MaxStraightSpeed))=0;
MaxStraightSpeed % Array of maximum speed at each straight sector!
%% Max Deceleration
% braking acceleration -a = -Fs/m
Deceleration = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    Deceleration(i) = (DecelerativeForce(CornerRadiusArray(i),ExitSpeed(i)))/(mass*g);
end
Deceleration;
%% Elapsed Time 
% From average speed through the current sector and the sector length, plus the sum of all previous sector elapsed times
% average speed = (entry + exit)/2
% time = speed*sector length
elapsedtime = 0;
ElapsedTime = zeros(length(sectorIndex),1);
for i = stop:length(sectorIndex)-1 % i = 1:length(sectorIndex)-1 for non-skidpad!
    avgspeed = (EntrySpeed(i)+ExitSpeed(i))/2;
    if avgspeed == 0
        ElapsedTime(i) = elapsedtime;
        continue;
    end
    slength = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2));
    ElapsedTime(i) = elapsedtime + slength/avgspeed;
    elapsedtime = elapsedtime + slength/avgspeed;
    
    if i == length(sectorIndex)-1
        ElapsedTime(i+1) = ElapsedTime(i);
    end
end
elapsedtime
ElapsedTime;
%% Distance Travelled
distance = zeros(length(sectorIndex),1);
totaldis = 0;
for i = 1:length(sectorIndex)-1
    if i == 1 || i == length(sectorIndex)
        continue;
    end
    
    slength = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2));
    distance(i) = distance(i-1) + slength;
    totaldis = totaldis + slength;
end
distance; % the matrix
totaldis; % value
%% Longitudinal and Lateral Acceleration
% Longitudinal: the difference in entry and exit speeds, and the sector length
longG = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)-1
    speeddif = EntrySpeed(i)-ExitSpeed(i);
    slength = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2));
    time = slength/speeddif;
    longG(i) = (speeddif/time)/g;
    
    if longG(i) > 2
     longG(i) = longG(i-1); % if radius is too big due to transitions, set it to previous radius
    end
end
longG;

% Lateral: average sector speed in the centripetal acceleration equation 
latG = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    speed = (EntrySpeed(i)+ExitSpeed(i)/2);
    latG(i) = (CentriAcceleration(CornerRadiusArray(i),speed))/g;
end
latG(isnan(latG))=0; % all NaN values become 0
latG;
%% Wheel Speed (MPH)
WheelSpeed = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    WheelSpeed(i) = (3600*ExitSpeed(i))/1609.34;
end
WheelSpeed;
%% Steering Angle
% 180-A
SteeringAngle = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    if i == 1
        SteeringAngle(i) = SteeringAngle(i+1);
    elseif i > length(sectorIndex) - 1
        SteeringAngle(i) = SteeringAngle(i-1);
    else
    a = sectorLength(skidpad(sectorIndex(i-1),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i-1),2),skidpad(sectorIndex(i+1),2)); % length from next sector to previous
    b = sectorLength(skidpad(sectorIndex(i),1),skidpad(sectorIndex(i+1),1),skidpad(sectorIndex(i),2),skidpad(sectorIndex(i+1),2)); % current sector to next
    c = sectorLength(skidpad(sectorIndex(i-1),1),skidpad(sectorIndex(i),1),skidpad(sectorIndex(i-1),2),skidpad(sectorIndex(i),2)); % previous to current
    SteeringAngle(i) = real((180 - angleA(a,b,c)));
    end
end
SteeringAngle;
%% Sectioning Track & Plot
sectionlength = nearest(length(sectorIndex)/6);
begin1 = 1;
end1 = sectionlength;
begin2 = end1 + 1;
end2 = begin2 + sectionlength;
begin3 = end2 + 1;
end3 = begin3 + sectionlength;
begin4 = end3 + 1;
end4 = begin4 + sectionlength;
begin5 = end4 + 1;
end5 = begin5 + sectionlength;
begin6 = end5 + 1;
end6 = length(skidpad);

Track1 = zeros(end1-begin1,2);
Track1 = skidpad(begin1:end1,1:2);

Track2 = zeros(end2-begin2,2);
Track2 = skidpad(begin2:end2,1:2);

Track3 = zeros(end3-begin3,2);
Track3 = skidpad(begin3:end3,1:2);

Track4 = zeros(end4-begin4,2);
Track4 = skidpad(begin4:end4,1:2);

Track5 = zeros(end5-begin5,2);
Track5 = skidpad(begin5:end5,1:2);

Track6 = zeros(end6-begin6,2);
Track6 = skidpad(begin6:end6,1:2);


nexttile
plot(Track1(:,1), Track1(:,2),'LineWidth',2.0);
hold on
plot(Track2(:,1), Track2(:,2),'LineWidth',2.0);
plot(Track3(:,1), Track3(:,2),'LineWidth',2.0);
plot(Track4(:,1), Track4(:,2),'LineWidth',2.0);
plot(Track5(:,1), Track5(:,2),'LineWidth',2.0);
plot(Track6(:,1), Track6(:,2),'LineWidth',2.0);
hold off
%% Plot
b = distance(:,1);
a = ElapsedTime(:,1);
i = 1:length(sectorIndex);

figure
plot(a,WheelSpeed(i));
hold on
xline(ElapsedTime(begin2));
xline(ElapsedTime(begin3));
xline(ElapsedTime(begin4));
xline(ElapsedTime(begin5));
%plot(a,Deceleration(i))
hold off
legend('Wheel Speed (MPH)') %,'Max Deceleration')
xlabel('Time')
xticks(0:10:length(sectorIndex))
axis([0 Inf 0 Inf])

figure
plot(a,Deceleration(i)');
hold on
plot(a,latG(i));
plot(a,longG(i))
xline(ElapsedTime(begin2));
xline(ElapsedTime(begin3));
xline(ElapsedTime(begin4));
xline(ElapsedTime(begin5));
hold off
legend('Max Deceleration','Lateral G','Longitudinal G')
xticks(0:10:length(sectorIndex))
axis([0 Inf 0 Inf])
xlabel('Time')

figure
plot(a,MaxCornerSpeed(i));
hold on
plot(a,MaxStraightSpeed(i));
plot(a,EntrySpeed(i));
plot(a,ExitSpeed(i))
xline(ElapsedTime(begin2));
xline(ElapsedTime(begin3));
xline(ElapsedTime(begin4));
xline(ElapsedTime(begin5));
hold off
legend('Max Corner Speed','Max Straight Speed','Entry Speed','Exit Speed')
xlabel('Time')
xticks(0:10:length(sectorIndex))
axis([0 Inf 0 Inf])
%% Functions
function [a] = sectorLength(x1,x2,y1,y2)
    a = ((x2 - x1)^2 + (y2 - y1)^2)^(1/2);
end

function [radius] = CornerRadius (a,b,c)
    A = acosd((b^2 + c^2 - a^2)/(2*b*c));
    radius = a/(2*sind(180 - A));
end

function [A] = angleA (a,b,c)
    A = acosd((b^2 + c^2 - a^2)/(2*b*c));
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

function [Fs] = DecelerativeForce (radius,velocity)
    global airdensity;
    global Af;
    global Cd;
    global tirefriction;
    global mass;
    global R;
    Fs = 0.5*airdensity*velocity^2*Af*Cd + (tirefriction^2*R^2 - mass^2*velocity^4/radius^2)^(1/2);
end

function [u] = MaxEntrySpeed (Fs, exitvelocity,s)
    global mass;
    u = (exitvelocity^2 + 2*s*Fs/mass)^(1/2);
end

function [centriacc] = CentriAcceleration (radius,velocity)
    centriacc = (velocity^2)/radius;
end