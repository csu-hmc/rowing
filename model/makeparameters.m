function [p] = makeparameters()
% makes a parameter structure for the rowerdynamics model
%
% Output:
%       p.........(structure) complete set of parameters for dynamics

	% Subject-specific parameters
	% These are the values that Qiangeng obtained from the first rowing experiment, so we can test the code.
	% They should be replaced (outside of this function) by values determined from each subject experiment.
	p.M                = 80;				     % body mass (kg)
	p.Xankle           =  78.8645/1000;          
	p.Yankle           =  521.6144/1000;   
	p.Xsprocket        = 1.0;					% needs to be estimated from mocap data!
	p.Ysprocket        = 1.0;                   % this one also      
	p.a                =  0.0286;                    
	p.b                =  786.6275/1000;            
	p.ShankLen         =  427.3084/1000;                 
	p.ThighLen         =  406.4490/1000;                  
	p.TrunkLen         =  428.7469/1000;                      
	p.UpparmLen        =  277.6224/1000;                  
	p.ForearmLen       =  228.077/1000;                    

	% These are model parameters
    p.g          = 9.80665;                     % gravity
	p.Kcrm       =  1e5;                        % cable stiffness (N/m)
	p.K          =  1e5;                        % seat stiffness (N/m)
	p.C          =  2*sqrt(p.M*p.K);            % seat damping (Ns/m), close to critical damping

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

end