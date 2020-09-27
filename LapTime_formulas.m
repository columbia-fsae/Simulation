%List of car specs
global weight;
global tirefriction;
global poweroutput;
global frontalarea;
global Cd;
global rho;
global R;
global mass;

weight = 1;
tirefriction = 1;
poweroutput = 1;
frontalarea = 1;
Cd = 1; % coefficient of drag
rho = 1; % density of air
R = 1; % normal force acting on the tire
r = 1; % radius of circle
mass = 1;

% Paramaters
% Fc = Thrust to produce centripetal force for given corner radius
% Fd = Thrust to overcome aerodynamic drag and maintain constant speed
% Fc = m*velocity^2/r Centripetal force
% r = radicus of circle
% velocity = arbitrary velocity;
% u = entry speed
% s =  distance travelled


%Test
a = endofsectorspeed(1,2)
b = decelerativeforce(1,2)
c = maxentryspeed(1,2,3)
d = maximumVelocity(1)
e = aerodynamicDrag(1)


% The speed of the car at the end of a sector of length s, given an entry speed u for the sector
function [sectorspeed] = endofsectorspeed (u,s)
    global poweroutput;
    global frontalarea;
    global Cd;
    global rho;
    global mass;
    sectorspeed = u^2 + (2*s*((poweroutput/u) - 0.5*rho*(u^2)*frontalarea*Cd))/mass;
end

% Force available for braking
function [Fs] = decelerativeforce (maxentryvel, r)
    global tirefriction;
    global frontalarea;
    global Cd;
    global rho;
    global R;
    global mass;
    Fs = 0.5*rho*maxentryvel^2*frontalarea*Cd + tirefriction^2*R^2 - (mass^2*maxentryvel^4)/r^2;
end

% Maximum entry speed u from which we could have braked
function [umax] = maxentryspeed (entryvelocity, s, Fs)
    global mass;
    umax = (entryvelocity^2 + (2*s*Fs)/mass)^(1/2);
end

% the maximum speed the car can go round a given radius curve
function[maxspeed] = maximumVelocity (r)
    global tirefriction;
    global frontalarea;
    global Cd;
    global rho;
    global R;
    global mass;
    maxspeed = (tirefriction*R)^2/((mass/r)^2 + (0.5*rho*frontalarea*Cd)^2);
end

function[Fd] = aerodynamicDrag (velocity)
    global frontalarea;
    global Cd;
    global rho;
    Fd = Cd*(0.5)*rho*velocity^2*frontalarea;
end


