%% Read in the endurance data

endurance_track_points = readmatrix('lat_long_endurance.csv');

%% Transform the data

X = [];

X = endurance_track_points(:,2)*pi/180*6371*1000.*cos((endurance_track_points(:,1)*(pi/180)));

X = X(:,1) - X(1,1);

Y = [];

Y = endurance_track_points(:,1)*pi*6371*1000/180;

Y = Y(:,1) - Y(1,1);

endurance_track = [X,Y];



%% Plot the Endurance Track

plot(endurance_track(:,1), endurance_track(:,2))

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

% Every 'interval' points constitute a sector
interval = 3; % stop = 87, divisible by 3
sectorIndex = 0:interval:length(endurance_track);
sectorIndex(1) = 1;

% Calculate index when timing starts, endurance_track SPECIFIC!
for i = 1:length(endurance_track)
    if endurance_track(i,1) ~= 0
        index = i;
        break;
    end
end
stop = nearest(index/interval);
%% Corner Radius
% Creates an array of corner radius using sector lengths
CornerRadiusArray = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    if i == 1
        CornerRadiusArray(i) = CornerRadiusArray(i+1);
    elseif i > length(sectorIndex) - 1
        CornerRadiusArray(i) = CornerRadiusArray(i-1);
    else
        a = sectorLength(endurance_track(sectorIndex(i-1),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i-1),2),endurance_track(sectorIndex(i+1),2)); % length from next sector to previous
        b = sectorLength(endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i),2),endurance_track(sectorIndex(i+1),2)); % current sector to next
        c = sectorLength(endurance_track(sectorIndex(i-1),1),endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i-1),2),endurance_track(sectorIndex(i),2)); % previous to current
        CornerRadiusArray(i) = real(CornerRadius(a,b,c)); % consider only the real part
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
        s = sectorLength(endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i),2),endurance_track(sectorIndex(i+1),2));
        MaxEntry(i) = MaxEntrySpeed(DecelerativeForce(CornerRadiusArray(i),EntrySpeed(i+1)),EntrySpeed(i+1),s);
        if MaxEntry(i) > EntrySpeed(i) % transition to braking
            ExitSpeed(i) = EntrySpeed(i+1);
        else % maximum braking
            EntrySpeed(i) = MaxEntry(i);
            ExitSpeed(i) = EntrySpeed(i+1);
        end
    end
    % calculate max entry (original exit speed)
    s = sectorLength(endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i),2),endurance_track(sectorIndex(i+1),2));
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
        slength = sectorLength(endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i),2),endurance_track(sectorIndex(i+1),2));
    else
        slength = 0;
    end
    MaxStraightSpeed(i) = EndofSectorSpeed(slength,EntrySpeed(i)); 
end
MaxStraightSpeed; % Array of maximum speed at each straight sector!
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
for i = stop:length(sectorIndex)-1 % i = 1:length(sectorIndex)-1 for non-endurance_track!
    avgspeed = (EntrySpeed(i)+ExitSpeed(i))/2;
    if avgspeed == 0
        continue;
    end
    slength = sectorLength(endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i),2),endurance_track(sectorIndex(i+1),2));
    elapsedtime = elapsedtime + slength/avgspeed;
end
elapsedtime;
%% Distance Travelled
distance = zeros(length(sectorIndex),1);
totaldis = 0;
for i = 1:length(sectorIndex)-1
    if i == 1 || i == length(sectorIndex)
        continue;
    end
    
    slength = sectorLength(endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i),2),endurance_track(sectorIndex(i+1),2));
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
    slength = sectorLength(endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i),2),endurance_track(sectorIndex(i+1),2));
    time = slength/speeddif;
    longG(i) = (speeddif/time)/g;
end
longG;

% Lateral: average sector speed in the centripetal acceleration equation 
latG = zeros(length(sectorIndex),1);
for i = 1:length(sectorIndex)
    speed = (EntrySpeed(i)+ExitSpeed(i)/2);
    latG(i) = (CentriAcceleration(CornerRadiusArray(i),speed))/g;
end
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
    a = sectorLength(endurance_track(sectorIndex(i-1),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i-1),2),endurance_track(sectorIndex(i+1),2)); % length from next sector to previous
    b = sectorLength(endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i+1),1),endurance_track(sectorIndex(i),2),endurance_track(sectorIndex(i+1),2)); % current sector to next
    c = sectorLength(endurance_track(sectorIndex(i-1),1),endurance_track(sectorIndex(i),1),endurance_track(sectorIndex(i-1),2),endurance_track(sectorIndex(i),2)); % previous to current
    SteeringAngle(i) = real((180 - angleA(a,b,c)));
    end
end
SteeringAngle;
%% Plot
a = distance(:,1);
i = 1:length(sectorIndex);

nexttile
plot(a,WheelSpeed(i))
hold on
plot(a,latG(i))
plot(a,longG(i))
plot(a,Deceleration(i))
hold off
legend('Wheel Speed (MPH)','latG','longG','Max Deceleration')
xlabel('Distance')
xticks(0:20:length(sectorIndex))
axis([0 Inf 0 Inf])

nexttile
plot(a,CornerRadiusArray(i))
hold on
plot(a,MaxCornerSpeed(i))
plot(a,MaxStraightSpeed(i))
hold off
legend('Corner Radius','Maximum Speed (Corner)','Maximum Speed (Straight)')
xticks(0:20:length(sectorIndex))
axis([0 Inf 0 20])
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