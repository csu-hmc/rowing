% rowingdyn.al
%
% Autolev code to compute inverse dynamics of the rower model, including the cable model
%
% The matlab function makerowerdynamics.m will transform this into a Matlab function that
% looks like this:
%
% function [f, df_dq, df_dqd, df_dqdd, df_dFc, stick] = rowerdynamics(q,qd,qdd,Fc,par)
%
% Author: Ton van den Bogert
% Last revised: 2/15/2018

PAUSE 0
AUTOZ ON
OVERWRITE ALL

%--------------------------------------------------------------------
%	Ground reference frame, units
%--------------------------------------------------------------------
Newtonian   Ground
UnitSystem  kg,meter,sec

%---------------------------------------------------
% Generalized coordinates
%---------------------------------------------------
%	q1....ankle angle
%	q2....knee angle
%	q3....hip angle
%	q4....shoulder angle
%	q5....elbow angle
MotionVariables' q{5}''

%---------------------------------------------------
% Body segment parameters
%---------------------------------------------------
Constants par__TrunkMass,   par__TrunkInertia,   par__TrunkCM,   par__TrunkLen
Constants par__ThighMass,   par__ThighInertia,   par__ThighCM,   par__ThighLen
Constants par__ShankMass,   par__ShankInertia,   par__ShankCM,   par__ShankLen
Constants par__UpparmMass,  par__UpparmInertia,  par__UpparmCM,  par__UpparmLen
Constants par__ForearmMass, par__ForearmInertia, par__ForearmCM, par__ForearmLen
Constants par__Xankle, par__Yankle
Constants par__Xsprocket, par__Ysprocket		% point where the cable comes out of the machine
Constants par__a, par__b						% seat path is Y = a*X + b
Constants par__Kseat, par__Cseat				% seat stiffness and damping
Constants par__Kcrm, par__K     				% cable stiffness and return spring stiffness
Constants par__g								% gravity
Constants Fc									% cable force (input from Matlab dynamics function)

%----------------------------------
% Define bodies and points
%----------------------------------
Bodies Trunk, Shank, Thigh, Upparm, Forearm
Points Hip, Knee, Ankle, Shoulder, Elbow, Wrist

%----------------------------------------------------------------
%    mass and inertia
%----------------------------------------------------------------
Mass     Trunk = par__TrunkMass
Mass     Shank = par__ShankMass
Mass     Thigh = par__ThighMass
Mass     Upparm = par__UpparmMass
Mass     Forearm = par__ForearmMass

Inertia  Trunk, 0, 0, par__TrunkInertia
Inertia  Shank, 0, 0, par__ShankInertia
Inertia  Thigh, 0, 0, par__ThighInertia
Inertia  Upparm, 0, 0, par__UpparmInertia
Inertia  Forearm, 0, 0, par__ForearmInertia

%---------------------------------------------------------------
% shank segment
%---------------------------------------------------------------
P_GroundO_Ankle> = par__Xankle*Ground1> + par__Yankle*Ground2>		% define the ankle position    
V_Ankle_Ground> = 0>										% and velocity
A_Ankle_Ground> = 0>										% and acceleration
Simprot(Ground, Shank,  3, q1);								% shank rotation
W_Shank_Ground>   = q1' * Ground3>					
ALF_Shank_Ground> = q1'' * Ground3>					
P_Ankle_ShankO> = par__ShankCM*Shank1>			% shank center of mass is on Shank X axis
V2pts(Ground, Shank, Ankle, ShankO);		
A2pts(Ground, Shank, Ankle, ShankO);	

%-----------------------------------------------------------------
% thigh segment
%-----------------------------------------------------------------
P_Ankle_Knee> = par__ShankLen * Shank1>
V2pts(Ground, Shank, Ankle, Knee);		
A2pts(Ground, Shank, Ankle, Knee);	
Simprot(Shank, Thigh, 3, q2)
W_Thigh_Ground>   = (q1' + q2') * Ground3>					
ALF_Thigh_Ground> = (q1'' + q2'') * Ground3>					
P_Knee_ThighO> = par__ThighCM*Thigh1>			% center of mass
V2pts(Ground, Thigh, Knee, ThighO);		
A2pts(Ground, Thigh, Knee, ThighO);	

%-----------------------------------------------------------------
% trunk segment
%-----------------------------------------------------------------
P_Knee_Hip> = par__ThighLen * Thigh1>
V2pts(Ground, Thigh, Knee, Hip);		
A2pts(Ground, Thigh, Knee, Hip);	
Simprot(Thigh, Trunk, 3, q3)
W_Thigh_Ground>   = (q1' + q2' + q3') * Ground3>					
ALF_Thigh_Ground> = (q1'' + q2'' + q3'') * Ground3>					
P_Hip_TrunkO> = -par__TrunkCM*Trunk1>			% trunk X axis points from shoulder to hip
V2pts(Ground, Trunk, Hip, TrunkO);		
A2pts(Ground, Trunk, Hip, TrunkO);	

%-----------------------------------------------------------------
% upper arm segment
%-----------------------------------------------------------------
P_Hip_Shoulder> = -par__TrunkLen * Trunk1>
V2pts(Ground, Trunk, Hip, Shoulder);		
A2pts(Ground, Trunk, Hip, Shoulder);	
Simprot(Trunk, Upparm, 3, q4)
W_Upparm_Ground>   = (q1' + q2' + q3' + q4') * Ground3>					
ALF_Upparm_Ground> = (q1'' + q2'' + q3'' + q4'') * Ground3>					
P_Shoulder_UpparmO> = par__UpparmCM*Upparm1>			% center of mass
V2pts(Ground, Upparm, Shoulder, UpparmO);		
A2pts(Ground, Upparm, Shoulder, UpparmO);	

%-----------------------------------------------------------------
% forearm segment
%-----------------------------------------------------------------
P_Shoulder_Elbow> = par__UpparmLen * Upparm1>
V2pts(Ground, Upparm, Shoulder, Elbow);		
A2pts(Ground, Upparm, Shoulder, Elbow);	
Simprot(Upparm, Forearm, 3, q5)
W_Forearm_Ground>   = (q1' + q2' + q3' + q4' + q5') * Ground3>					
ALF_Forearm_Ground> = (q1'' + q2'' + q3'' + q4'' + q5'') * Ground3>					
P_Elbow_ForearmO> = par__ForearmCM*Forearm1>			% center of mass
V2pts(Ground, Forearm, Elbow, ForearmO);		
A2pts(Ground, Forearm, Elbow, ForearmO);	

%--------------------------------------------------------------------
%   gravity and the force of spring and damper that supports the seat
%--------------------------------------------------------------------   
gravity(-par__g*Ground2>)
PP> = (par__a*Ground1>  - Ground2>) / sqrt(1+par__a*par__a)        % a unit vector that is perpendicular to the line 
d = dot(P_GroundO_Hip> - par__b*Ground2>,PP>)            % perpendicular distance from Hip to the line
force_Hip> += -par__Kseat*d*PP> - par__Cseat*dt(d)*PP>       % spring and damper force perpendicular to the line and

%-------------------------------------------------------
% compute wrist position and cable force
%-------------------------------------------------------
P_Elbow_Wrist> = par__ForearmLen*Forearm1>			
V2pts(Ground, Forearm, Elbow, Wrist);		% velocity is needed for equations of motion
cable> = Vector(Ground,par__Xsprocket,par__Ysprocket,0) - P_GroundO_Wrist>   % vector from wrist to sprocket
L = mag(cable>);						    % distance between wrist and sprocket 
Ldot = dt(L);
force_Wrist> += Fc * unitvec(cable>)		% force vector acting on the wrist

%----------------------------------------------------------
% x and y coordinates of the joints and sprocket, stored in a 6x2 matrix
%----------------------------------------------------------
stick = [	par__Xankle,    			par__Yankle ; &
			dot(P_GroundO_Knee>,  		Ground1>) , dot(P_GroundO_Knee>,  		Ground2>) ; &
			dot(P_GroundO_Hip>,  		Ground1>) , dot(P_GroundO_Hip>,  		Ground2>) ; &
			dot(P_GroundO_Shoulder>,    Ground1>) , dot(P_GroundO_Shoulder>,    Ground2>) ; &
			dot(P_GroundO_Elbow>,    	Ground1>) , dot(P_GroundO_Elbow>,	    Ground2>) ; &
			dot(P_GroundO_Wrist>,    	Ground1>) , dot(P_GroundO_Wrist>,	    Ground2>) ; &
			par__Xsprocket,             par__Ysprocket]
			
%----------------------------------------------------------------
% generate the outputs
%----------------------------------------------------------------
f = -fr()-frstar()			% the five torques

% vectors needed to define Jacobian matrices
q   = [q1,   q2,   q3,   q4,   q5]
qd  = [q1',  q2',  q3',  q4',  q5']
qdd = [q1'', q2'', q3'', q4'', q5'']

% the Jacobians
df_dq   = ZEE(D(f,q))
df_dqd  = ZEE(D(f,qd))
df_dqdd = ZEE(D(f,qdd))
df_dFc = ZEE(D(f,Fc))
dL_dq = ZEE(D(L,q))
dLdot_dq = ZEE(D(Ldot,q))

Encode f, df_dq, df_dqd, df_dqdd, df_dFc, L, dL_dq, dLdot_dq, stick
Code Algebraic() tmp.m

EXIT
