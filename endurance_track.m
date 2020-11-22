%% Read in data

endurance_track_points = readmatrix('lat_long_endurance.csv');

%% Transform the data

X = [];

X = endurance_track_points(:,2)*pi/180*6371*1000.*cos((endurance_track_points(:,1)*(pi/180)));

X = X(:,1) - X(1,1);

Y = [];

Y = endurance_track_points(:,1)*pi*6371*1000/180;

Y = Y(:,1) - Y(1,1);

endurance_track = [X,Y];



%% Plot the Data

plot(endurance_track(:,1), endurance_track(:,2))