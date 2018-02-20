function mytest
    % runs a forward dynamic simulation of the rower model to test the dynamics code
    global model
    tic
    
    addpath('../tools');
    [~,~,~, model.parameters] = datainterp2;
    
    % test the derivatives of dynfun.m
    disp('Testing dynfun.m');
    x = rand(13,1);
    xd = rand(13,1);
    u = rand(5,1);
    [f, df_dx, df_dxd, df_du, L, dL_dq] = dynfun(x, xd, u);
    df_dx_num = zeros(13,13);
    df_dxd_num = zeros(13,13);
    df_du_num = zeros(13,5);
    dL_dq_num = zeros(1,5);
    hh = 1e-7;
    for i = 1:13
        save = x(i);
        x(i) = x(i) + hh;
        [fh,~,~,~,Lh] = dynfun(x, xd, u);
        df_dx_num(:,i) = (fh - f)/hh;
        if (i>=2) && (i<=6)
            dL_dq_num(1,i-1) = (Lh - L)/hh;
        end
        x(i) = save;
        %---------------------
        save = xd(i);
        xd(i) = xd(i) + hh;
        fh = dynfun(x, xd, u);
        df_dxd_num(:,i) = (fh - f)/hh;
        xd(i) = save;
    end
    for i = 1:5
        save = u(i);
        u(i) = u(i) + hh;
        fh = dynfun(x, xd, u);
        df_du_num(:,i) = (fh - f)/hh;
        u(i) = save;
    end
	fprintf('Max. error in df_dx: ');       matcompare(df_dx, df_dx_num);
	fprintf('Max. error in df_dxd: ');       matcompare(df_dxd, df_dxd_num);
	fprintf('Max. error in df_du: ');       matcompare(df_du, df_du_num);
	fprintf('Max. error in dL_dq: ');       matcompare(dL_dq, dL_dq_num);
   
    
    % test the derivatives of rowerdynamics.m
    disp('Testing rowerdynamics.m...');
    q = rand(5,1);
    qd = rand(5,1);
    qdd = rand(5,1);
    Fc = rand(1,1);
    [f, df_dq, df_dqd, df_dqdd, df_dFc, L, dL_dq, dLdot_dq] = rowerdynamics(q,qd,qdd,Fc,model.parameters);
    df_dq_num = zeros(5,5);
    df_dqd_num = zeros(5,5);
    df_dqdd_num = zeros(5,5);
    df_dFc_num = zeros(5,1);
    dL_dq_num = zeros(1,5);
    dLdot_dq_num = zeros(1,5);
    Ldot = dL_dq * qd;
    hh = 1e-7;
    for i = 1:5
        save = q(i);
        q(i) = q(i) + hh;
        [fh,~,~,~,~,Lh,dL_dqh] = rowerdynamics(q,qd,qdd,Fc,model.parameters);
        df_dq_num(:,i) = (fh - f)/hh;
        dL_dq_num(:,i) = (Lh - L)/hh;
        Ldoth = dL_dqh * qd;
        dLdot_dq_num(:,i) = (Ldoth - Ldot)/hh;
        q(i) = save;
        %-----------------------
        save = qd(i);
        qd(i) = qd(i) + hh;
        fh = rowerdynamics(q,qd,qdd,Fc,model.parameters);
        df_dqd_num(:,i) = (fh - f)/hh;
        qd(i) = save;
        %-----------------------
        save = qdd(i);
        qdd(i) = qdd(i) + hh;
        fh = rowerdynamics(q,qd,qdd,Fc,model.parameters);
        df_dqdd_num(:,i) = (fh - f)/hh;
        qdd(i) = save;
    end
    save = Fc;
    Fc = Fc + hh;
    fh = rowerdynamics(q,qd,qdd,Fc,model.parameters);
    df_dFc_num(:,1) = (fh - f)/hh;
    Fc = save;
	fprintf('Max. error in df_dq: ');       matcompare(df_dq, df_dq_num);
	fprintf('Max. error in df_dqd: ');      matcompare(df_dqd, df_dqd_num);
 	fprintf('Max. error in df_dqdd: ');     matcompare(df_dqdd, df_dqdd_num);
 	fprintf('Max. error in df_dFc: ');      matcompare(df_dFc, df_dFc_num);
 	fprintf('Max. error in dL_dq: ');       matcompare(dL_dq, dL_dq_num);
 	fprintf('Max. error in dLdot_dq: ');	matcompare(dLdot_dq, dLdot_dq_num);
    keyboard
	
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
    legend('q1','q2','q3','q4','q5');
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('angle (deg)','fontweight','bold','fontsize',10);
    title('Joint Angles','fontweight','bold','fontsize',10)
    figure(3)
    plot(tt, f, 'LineWidth',2);
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('wrist force (N)','fontweight','bold','fontsize',10);
    title('Force','fontweight','bold','fontsize',10)
    
    figure(4)
    plot(tt, yy(:,1), 'LineWidth',2);
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('fwpos (m)','fontweight','bold','fontsize',10);
    title('flywheel position','fontweight','bold','fontsize',10)
    
    figure(5)
    plot(tt, yy(:,7), 'LineWidth',2);
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('fwvelo (m/s)','fontweight','bold','fontsize',10);
    title('flywheel velocity','fontweight','bold','fontsize',10)

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
%====================================================================
    function matcompare(a,b)
	% compares two matrices and prints element that has greatest difference
    if (size(a,1) == 1)
        irow = 1;
        [maxerr,icol] = max(abs(a-b));
    else
        [maxerr,irow] = max(abs(a-b));
        [maxerr,icol] = max(maxerr);
        irow = irow(icol);
    end
	fprintf('%9.6f at %d %d (%9.6f vs. %9.6f)\n', full(maxerr), irow, icol, full(a(irow,icol)), full(b(irow,icol)));
    end
