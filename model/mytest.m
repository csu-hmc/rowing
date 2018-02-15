 function mytest
% runs a forward dynamic simulation of the rower model to test the dynamics code
	global model
    [~,~,~, model.parameters] = datainterp2;
	
	% choose a suitable initial condition: realistic joint angles, and choose flywheel position y(1)
	% to be equal to the L that comes from these joint angles
	% and make all velocities zero
    y0 = [ 0.1 ;[86.6804 ; 121.4658 ;  60.3518 ;  61.3895 ;  12.4060] * pi/180;  0   ; 0; 0; 0; 0;0; 0];   % initial condition () 
    yp0 = zeros(13,1);			                        % initial guess for state derivatives at t=0
    %fixed_y0 = [0 1 1 1 1 1 1 0 0 0 0 0 0];
    %fixed_yp0 = [];
    fixed_y0 = [];
    fixed_yp0 = [];
    T = 0:0.01:2.0;                                     % simulation time
    [y0,yp0] = decic(@dyn,T(1),y0,fixed_y0,yp0,fixed_yp0);	% find consistent initial conditions
    [tt,yy]  = ode15i(@dyn, T, y0,yp0);			% simulate using ode15i

    % make the AVI file
    disp('making AVI file...')
    q = yy(:,[ 2 3 4 5 6]);						    % extract joint angle trajectories (states 2-6)
    xfw = yy(:,1);
    f = yy(:,13);
    figure(1);
    animate(q,model.parameters);  		
    
	% make plots
    figure(2)
    plot(tt, q*180/pi, 'LineWidth',2);
    legend('q1','q2','q3','q4','q5','f');
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('angle (deg)','fontweight','bold','fontsize',10);
    title('Joint Angles','fontweight','bold','fontsize',10)
    figure(3)
    plot(tt, f, 'LineWidth',2);
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('force (N)','fontweight','bold','fontsize',10);
    title('Force','fontweight','bold','fontsize',10)
    
    figure(4)
    plot(tt, yy(:,1), 'LineWidth',2);
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('fwpos (m)','fontweight','bold','fontsize',10);
    title('fwpos','fontweight','bold','fontsize',10)
    
    figure(5)
    plot(tt, yy(:,7), 'LineWidth',2);
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('fwvelo (m)','fontweight','bold','fontsize',10);
    title('fwvelo','fontweight','bold','fontsize',10)
    

end
%=======================================================================
function f =  dyn(t,y, ydot)

    % extract q, qdot from y
   	q = y(2:6);
	qd = y(8:12);
   
	% generate joint torques with a PD controller
    % controller is turned on at tstart
	% simulation 1: tstart = 10, model should just fall down
	% simulation 2, use initial angles as the setpoint, and tstart = 0
	% simulation 3, use a leaning back setpoint, to make the human pull
	% simulation 4, use tstart = 0.5, model should fall down and come back up
	qsetpoint = [152.5304  ; 18.1593;  114.5404 ; -23.0064;  102.3893]  * pi/180;	
	tstart = 0.4;
	if t < tstart                                         
		kp = 0;
		kd = 0;
	else
		kp = 600;
		kd = 50;
	end
	tau = -kp*(q-qsetpoint) - kd*qd;
    
    % evaluate the implicit dynamics function
	f = dynfun(y,ydot,tau);

	if toc > 1.0
        fprintf('simulating...  t = %8.3f\n',t);
        tic
	end
    
end
%=======================================================================