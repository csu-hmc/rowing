function [q,t2,p] = bodyangles
    % better fucntion name the part that produce p and q
    
    % load X2016_03_17_trial2_markers
    load Mdata2
    
    %choose the averaged trial number (to underestand how the data is averaged visit "how to make data note")
    d = MDATA;

    % Calculating the knee angle(q2) using 3 markers: RLEK, RLM, RGTRO
    knee     = d(:,[33 34])/1000;          % RLEK: right lateral epicondyle of the knee x:31 Y:32 Z:33 (add 1 to column numbers)
    ankle    = d(:,[27 28])/1000;          % RLM : right lateral malleolus of the ankle x:25 Y:26 Z:27
    hip      = d(:,[39 40])/1000;          % RGTRO : right greater trochanter of femur  x:37 Y:38 Z:39
    shoulder = d(:,[63 64])/1000;          % RSHO : right acromian of shoulder x:61 Y:62 Z:63
    elbow    = d(:,[66 67])/1000;          % RLEE : right lateral elbow x:64 Y:65 Z:66
    wrist    = d(:,[69 70])/1000;          % RLW : right lateral wrist x:67 Y:68 Z:69
    t2 = d(:,1);                           % defining the step time

    f = find(~isnan(knee(:,1)));   %find the nan index in the knee data

    %remove the NAN numbers from the data and separate Y and Z cordinate for each marker
    kneeY = knee(f,1);
    kneeZ = knee(f,2);
    ankleY = ankle(:,1);
    ankleZ = ankle(:,2);
    hipY = hip(:,1);
    hipZ = hip(:,2);
    shoulderY = shoulder(:,1);
    shoulderZ = shoulder(:,2);
    elbowY = elbow(:,1);
    elbowZ = elbow(:,2);
    wristY = wrist(:,1);
    wristZ = wrist(:,2);

    %new time vector based on the dimention of
    newt = t2(f);
    kneeY = interp1(newt,kneeY,t2);
    kneeZ = interp1(newt,kneeZ,t2);

    % shank angle
    theta_shank = atan2((kneeY - ankleY),(kneeZ - ankleZ)); %calculating shank angle
    theta_thigh = atan2((hipY - kneeY),(hipZ - kneeZ)); %calculating femur angle
    q1 = theta_shank;

    % knee angle
    theta_thigh = unwrap(theta_thigh);
    q2 =  2*pi + theta_thigh - theta_shank;

    % hip angle
    theta_upperbody = atan2((shoulderY - hipY),(shoulderZ - hipZ)); %calculating
    q3 = theta_upperbody - (theta_thigh + pi);%
    q3 = unwrap(q3);

    % arm angle
    theta_arm = atan2((elbowY - shoulderY),(elbowZ - shoulderZ)); %calculating
    q4 = theta_arm - (theta_upperbody - pi) ;
    q4 = unwrap(q4);
%--------------------------------------------------------------------------

    % forearm angle
    theta_forearm = atan2((wristY - elbowY),(wristZ - elbowZ )); %calculating
    q5 = theta_forearm - theta_arm;
    
    %preparing the data (positions, velocities, accelerations)
    q = [q1 q2 q3 q4 q5];

    plot(q*180/pi)
    title('joint angles')
    legend('q1','q2','q3','q4','q5');
    set(gcf,'renderer','painters');

    % determine model parameters
    p.ShankLen = mean(sqrt((kneeY-ankleY).^2+(kneeZ-ankleZ).^2));
    p.ThighLen = mean(sqrt((hipY-kneeY).^2+(hipZ-kneeZ).^2));
    p.TrunkLen = mean(sqrt((shoulderY-hipY).^2+(shoulderZ-hipZ).^2));
    p.UpparmLen = mean(sqrt((elbowY-shoulderY).^2+(elbowZ-shoulderZ).^2));
    p.ForearmLen = mean(sqrt((wristY-elbowY).^2+(wristZ-elbowZ).^2));
    
    % calculate the coordinate of ankle point, and assume it is fixed
    p.Xankle = mean(ankleZ);
    p.Yankle = mean(ankleY);
    
    % do the linear polyfit of hip motion track for the alpha angle
    coeff = polyfit(hipZ,hipY,1);
    p.a=coeff(1,1);
    p.b=coeff(1,2); %FIT A STAIGHT LINE THROUGH A LINEAR REGRETION
    
    % define scale factor to convert kg to Mg and N to kN
    scale = 0.001;
	p.M                = scale * 80;				    % body mass (kg)
      
    % determins sprocket position based on the most forward position of the
    % wrist (IN FUTURE WORK WE NEED A MARKER)
    %i = find(max(wristZ));
    [p.Xsprocket,i] = max(wristZ);
    p.Xsprocket = max(wristZ) + 0.1;
	p.Ysprocket     = wristY(i);                       
                 
	% These are model parameters
    p.g          = 9.80665;                     % gravity(N/kg)
	p.Kseat          =  scale * 10^5;           % seat stiffness (kN/m)
	p.Cseat          =  2*sqrt(p.M*p.Kseat);    % seat damping (kNs/m), close to critical damping
    
    % rowing machine parameters from parameter identification
    p.C = scale * 89.68;             % flywheel damping coefficient kN/(m/s)^2
	p.m = scale * 539.6;             % flywheel equivalent mass
  	p.Kcrm = scale * 28386;          % stiffness of cable and ratchet mechanism (kN/m) (CHECK VALUE IN PARAMETER IDENTIFICATION
    p.K =  scale * 12.97 ;			 % shock cord stiffness (kN/m)
    p.L0 = 0.2;                      % the wrist-sprocket distance at which shock cord has no force
    
	% body segment parameters calculated using the tables from Winter's book
	p.ShankMass        =  0.0465*2*p.M;                
	p.ThighMass        =  0.1*2*p.M;                   
	p.TrunkMass        =  0.497*p.M;                   
	p.UpparmMass       =  0.028*2*p.M;                 
	p.ForearmMass      =  0.016*2*p.M;                 
	p.ShankInertia     =  p.ShankMass*(0.302*p.ShankLen)^2; 
	p.ThighInertia     =  p.ThighMass*(0.323*p.ThighLen)^2; 
	p.TrunkInertia     =  p.TrunkMass*(0.5*p.TrunkLen)^2;   
	p.UpparmInertia    =  p.UpparmMass*(0.322*p.UpparmLen)^2;
	p.ForearmInertia   =  p.ForearmMass*(0.303*p.ForearmLen)^2;
	p.ShankCM          =  (1-0.433)*p.ShankLen;                
	p.ThighCM          =  (1-0.433)*p.ThighLen;               
	p.TrunkCM          =  (1-0.500)*p.TrunkLen;                 
	p.UpparmCM         =  0.436*p.UpparmLen;                   
	p.ForearmCM        =  0.430*p.ForearmLen;                    
    %q = [q1,q2,q3,q4,q5];						    % extract joint angle trajectories (states 1-5)
    %figure(9);
    %animate(q,p);

 

end