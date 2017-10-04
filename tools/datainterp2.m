
function [dSPACEdata, qdata, t, p] = datainterp2

    global model
    load('Mdata2.mat');
    load('data2richter.mat');    


    model.r = 0.0135;
    HMCdata.tvect = DATA(:,1);  %time
    HMCdata.vel   = DATA(:,21); %handle velocity  
    HMCdata.force = DATA(:,20); %force        
    HMCdata.fwvel = DATA(:,19); %flywheel velocity
    HMCdata.fwvel = HMCdata.fwvel*model.r;

    % load the Concept 2 force-velocity data and resample everything on a
    % time grid with N points
    HMCdata.tvect = HMCdata.tvect - HMCdata.tvect(1);		% ensure that time starts at zero
    HMCdata.T = HMCdata.tvect(end);  % duration of the rowing cycle
%     HMCdata.force = HMCdata.force/1000;	% we do everything in kN

%     HMCdata.force = HMCdata.force;


    [qdata,t2,p] = bodyangles;

    % One rowing cycle
    frames = 1086:1278;
    t2 = t2(frames);
    qdata = qdata(frames,:);
    
    % Resample the dspace data at the same time points
    dSPACEdata=[HMCdata.vel,HMCdata.fwvel,HMCdata.force];  
    dSPACEdata = interp1(HMCdata.tvect, dSPACEdata, t2+0.62);
    HMCdata.vel = dSPACEdata(:,1);
    HMCdata.fwvel = dSPACEdata(:,2);
    HMCdata.force = dSPACEdata(:,3);
    t = t2 - t2(1);
    
end