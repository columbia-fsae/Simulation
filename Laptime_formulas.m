%% Car specs
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

% Test
sectorlength = sectorLength(1,2,1,2)
radius = CornerRadius(2,2,2)

maxcornerspeed = MaximumCornerSpeed(1)

dragforce = AeroDrag (1); % parameter for CombinedForce
centriforce = CentripetalForce (1,1); % parameter for CombinedForce
combinedforce = CombinedForce(dragforce,centriforce)

endfosectorspeed = EndofSectorSpeed (1,1)

Fs = DecelerativeForce (1,1) % parameter for MaxEntrySpeedBrake
Fb = ForceAvailableBraking (1,1) % parameter for MaxEntrySpeedBrake
u = MaxEntrySpeedBrake (Fs,1,1)

maxcornerspeedwing = MaxCornerSpeedWing (1) % parameter for StraightlineSpeedWing
straightlinespeed = StraightlineSpeedWing (1,1) % parameter for StraightlineSpeedWing
Fsbrake = MaxBrakingForceWing (1,1)

%% Functions
% Corner Radius
function [radius] = CornerRadius (a,b,c)
    A = acosd((b^2 + c^2 - a^2)/(2*b*c));
    radius = a/(2*sind(180 - A));
end

function [a] = sectorLength(x1,x2,y1,y2)
    a = ((x2 - x1)^2 + (y2 - y1)^2)^(1/2);
end

% Maximum corner speed
function [maxcornerspeed] = MaximumCornerSpeed(radius)
    global R;
    global tirefriction;
    global mass;
    global airdensity;
    global Af;
    global Cd;    
    maxcornerspeed = ((tirefriction*R)^2/((mass/radius)^2 + (0.5*airdensity*Af*Cd)^2))^(1/4);
end

    % Combined force
    function [combinedforce] = CombinedForce(dragforce,centriforce)
        combinedforce = (dragforce^2 + centriforce^2)^(1/2);
    end
    
        % Aerodynamic Drag
        function [dragforce] = AeroDrag (velocity)
            global Cd;
            global airdensity;
            global Af;
            dragforce = Cd*(05)*airdensity*velocity^2*Af;
        end

        % Centripetal force
        function [centriforce] = CentripetalForce (radius,velocity)
            global mass;
            centriforce = (mass*velocity^2)/radius;
        end
    
% Acceleration from rest, straight sector, s = distance travelled,
% u = entry speed
function [endofsectorspeed] = EndofSectorSpeed (s,u)
    global poweroutput;
    global airdensity;
    global Af;
    global mass;
    global Cd;
    endofsectorspeed = (u^2 + s*2*((poweroutput/u)-05*airdensity*u^2*Af*Cd)/mass)^(1/2);
end
    
% Combining Acceleration and Corners
    % Force available for braking
    function [Fb] = ForceAvailableBraking (radius,velocity)
        global tirefriction;
        global R;
        global mass;
        Fb = (tirefriction^2*R^2 - mass^2*velocity^4/radius^2)^(1/2);
    end

    % Total decelerative force Fs
    function [Fs] = DecelerativeForce (radius,velocity)
        global airdensity;
        global Af;
        global Cd;
        global tirefriction;
        global mass;
        global R;
        Fs = 05*airdensity*velocity^2*Af*Cd + (tirefriction^2*R^2 - mass^2*velocity^4/radius^2)^(1/2);
    end
    
    % Maximum entry speed u from which we could have braked
    function [u] = MaxEntrySpeedBrake (Fs, velocity,s)
        global mass;
        u = (velocity^2 + 2*s*Fs/mass)^(1/2);
    end
    
% Wings, downforce and drag
    % Maximum corner speed with wings
    function [maxcornerspeed] = MaxCornerSpeedWing (radius)
        global tirefriction;
        global mass;
        global g;
        global airdensity;
        global Af;
        global Cd;
        global Aw;
        global Cwd;
        global Cl;
        maxcornerspeed = (tirefriction*mass*g)/(((mass/radius)^2 + (05*airdensity*(Af*Cd + Aw*Cwd))^2)^(1/2) - 0.5*tirefriction*airdensity*Aw*Cl);
    end
    
    % Straight line speed, with wings
    function [straightlinespeed] = StraightlineSpeedWing (u,s)
        global poweroutput;
        global airdensity;
        global Af;
        global Aw;
        global Cwd;
        global mass;
        global Cd;
        straightlinespeed = (u^2 + 2*s*((poweroutput/u) - 0.5*airdensity*u^2*(Af*Cd + Aw*Cwd))/mass)^(1/2);
    end
    
    %Maximum braking force, with wings
    function [Fs] = MaxBrakingForceWing (radius, velocity)
        global airdensity;
        global Af;
        global Cd;
        global Aw;
        global Cwd;
        global tirefriction;
        global mass;
        global g;
        global Cl;
        Fs = 0.5*airdensity*velocity^2*(Af*Cd + Aw*Cwd) + (tirefriction^2*(mass*g + 0.5*airdensity*velocity^2*Aw*Cl)^2 - mass^2*velocity^4/radius^2)^(1/2);
    end