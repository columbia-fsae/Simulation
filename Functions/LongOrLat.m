function mode = LongOrLat()
% @editor Edward Li
% @UNI: wl2787
% @date: Nov. 23, 2020
% 
% LongOrLat.m - Function to determine whether the user wants to process
% lateral or longitudinal data. 
 
answer = questdlg('Which type of data would you like to process?',...
    'Data type', 'Longitudinal','Lateral','Longitudinal');

switch answer
    case 'Longitudinal'
        mode = 1;
    case 'Lateral'
        mode = 2;
end

