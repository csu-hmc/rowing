function test
% runs a forward dynamic simulation of the rower model to test the dynamics code
	global model
	
    tic
	model.parameters = makeparameters();			% set the parameter values
	
    y0 = [120; 70; 70; 80; 20; 0; 0; 0; 0; 0]*pi/180;   % initial condition
    yp0 = zeros(10,1);			                        % initial guess for state derivatives at t=0
    T = 0:0.01:2.0;                                     % simulation time
    [y0,yp0] = decic(@dynfun,T(1),y0,[],yp0,[]);	% find consistent initial conditions
    [tt,yy]  = ode15i(@dynfun, T, y0,yp0);			% simulate using ode15i
	
    % make the AVI file
    disp('making AVI file...')
    q = yy(:,[1 2 3 4 5]);						    % extract joint angle trajectories (states 1-5)
    figure(1);
    animate(q,model.parameters);  		
    
	% make plots
    figure(2)
    plot(tt, q*180/pi);
    legend('q1','q2','q3','q4','q5');
    xlabel('time (s)');
    ylabel('angle (deg)');
    title('Joint Angles')
end
%=======================================================================
function f = dynfun(t,y,ydot)
% dynamics of the rower model, in implicit form f(y,ydot,t) = 0
% where y is (q,qdot)
	global model

	f = zeros(10,1);		% f will be a 10x1 matrix

    % extract q, qdot, qdotdot from y and ydot
   	q = y(1:5);
	qd = y(6:10);
	qdd = ydot(6:10);
	
	% generate joint torques with a PD controller
    % controller is turned on at t=0.5 s
	qsetpoint = [120; 70; 70; 80; 10] * pi/180;		
	if t < 0.5                                         
		kp = 0;
		kd = 0;
	else
		kp = 2000;
		kd = 10;
	end
	tau = -kp*(q-qsetpoint) - kd*qd;
	
	% equations 1-5 of the implicit dynamics: derivative of q must be equal to qdot
	f(1:5) = ydot(1:5) - y(6:10);
	
	% equations 6-10: inverse dynamic torque must be equal to the applied torque
	xfw = 100;				% move the flywheel far to the right, so no cable force will be generated
    ff = rowerdynamics(q,qd,qdd,xfw,model.parameters);  % do the inverse dynamic calculation
	f(6:10) = tau - ff(1:5);                            % first 5 elements of ff are the inverse dynamic torques

    if toc > 1.0
        fprintf('simulating...  t = %8.3f\n',t);
        tic
    end
    
end
%=======================================================================