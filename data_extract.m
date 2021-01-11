clear;
clc;
close all;

addpath(fullfile(pwd,'Functions'));

datamode = LongOrLat(); % Choose longetudinal or lateral mode
[fileid, pathname] = uigetfile({'*.mat;*.dat'},'File Selector');
if( isa(fileid, 'double') && isequal(fileid, 0) )
    % No file was chosen, so we return
    return
end

[AT, ET, FX, FY, FZ, IA, MX, MZ, N, NFX, NFY, P, RE, RL, RST, SA, SR, TSTC, TSTI, TSTO, V]...
    = ImportRawData( fullfile(pathname, fileid) );

%disp(FX);
%disp(class(FX));
X = [MX MZ FX FY FZ IA];
writematrix(X, "data.csv");