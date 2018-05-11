
function result = optimize(problem)

% Input: problem, a structure with the following fields:
%   filename...(scalar) data file
%   N..........(scalar) number of collocation points 
%   Wtrack.....(scalar) weight of tracking term 
%   Weffort....(scalar) weight of effort term in objective
%   X..........(scalar) initial guess (default: zeros)
    
   global model 
   load Simresult

   [dSPACEdata, qdata, t, model.parameters] = datainterp2;

   T = t(end); %duration of one rowing cycle
   
   % decode the problem
   N = problem.N;      % number of collocation points
   model.N = N;
   model.tracking = problem.tracking;
   model.Wtrack  = problem.Wtrack;
   model.Weffort = problem.Weffort;
   if isfield(problem,'task')
       model.task.Lmin = problem.task.Lmin;
       model.task.Lmax = problem.task.Lmax;
   end

   
   model.h = T/(N-1);    % time step for direct collocation
   h =  model.h;
   tdc = (0:h:T)';    % time points for direct collocation
   model.Len = zeros(1,model.N);
   
   % resample the tracking data to the DC time points 
   qdata = interp1(t, qdata, tdc);
   dSPACEdata = interp1(t, dSPACEdata, tdc);
   veldata = dSPACEdata(:,1);
   fwveldata = dSPACEdata(:,2);
   forcedata = dSPACEdata(:,3);
   
   plot(qdata*180/pi)
   title('joint angles')
   legend('q1','q2','q3','q4','q5');
   % store the tracking data in global variables
   model.time =  tdc;
   %     model.data = [qdata fwveldata forcedata];   % add mocap data (angles) as extra columns
   if model.tracking == 1
       model.data = [qdata];   % add mocap data (angles) as extra columns
   elseif  model.tracking == 2
       model.data = [qdata forcedata];   % add mocap data (angles) as extra columns
   elseif model.tracking == 3
       model.data = [qdata fwveldata forcedata];   % add mocap data (angles) as extra columns
   elseif model.tracking == 4
       model.data = qdata(:,5);
   else
       model.data = [qdata(:,4:5)];
   end
   
   model.dataSD = std(model.data);
   
   if model.tracking == 1
       model.itrack = [2 3 4 5 6];       % the state variables to track (model.itrack=problem.irack so it should go to the script)
   elseif model.tracking == 2
       model.itrack = [2 3 4 5 6 13];       % the state variables to track (model.itrack=problem.irack so it should go to the script)
   elseif model.tracking == 3
       model.itrack = [2 3 4 5 6 7 13];
   elseif model.tracking == 4
       model.itrack = [6];
   else
       model.itrack = [5 6];
   end
   
   model.ntrack = numel(model.itrack);
   
   model.discretization = problem.discretization;  % Backward Euler(BE) or Midpoint Euler(ME) discretization
   model.N = problem.N;
   model.Wtrack  = problem.Wtrack;
   model.Weffort = problem.Weffort;
   model.cgain = problem.cgain;
   model.dyngain = problem.dyngain;
%    if isfield(problem,'task')
%    model.cablecnst = problem.cablecnst;
%    else
%    model.cablecnst =  0;
%    end
   model.umax = [ 6.11,6.00,6.071,1.3557,1.1164]';
  % model.umax = [ 1,1,1,1,1]';


   
   
   % define the state variables x, controls u, and their bounds
   % x = [x_fw, q1 ...q5, v_fw, qd1...qd5, F(kn)]
   % u = [M1(knm)...M5];
   model.nx = 13;
   model.nu = 5;
   model.np = problem.np;
   
   %before scaling:
   % x_lb = [0    0     -2*pi    0   -pi/2       0   0  -10 -10 -10 -10 -10  0]';
   % x_ub = [3  2*pi   2*pi  2*pi   pi/3      2*pi  20  20  20  20  20  20  500 ]';
   % u_lb = [-700; -700 ;-700 ;-700 ;-700];
   % u_ub =  [700 ;700 ;700 ;700 ;700];
   
   % when scaled
   %     x_lb = [0    0     -2*pi    0   -pi/2       0   0  -20 -10 -10 -10 -10  0 ]';
   %     x_ub = [3  2*pi   2*pi  2*pi   pi/3      2*pi  20  20  20  20  20  20  5 ]';
   x_lb = [0    80*pi/180     20*pi/180    50*pi/180   -40*pi/180       10*pi/180   0  -20 -10 -10 -10 -10  0 ]';
   x_ub = [3   160*pi/180     130*pi/180   120*pi/180   80*pi/180      115*pi/180  20  20  20  20  20  20  0.42 ]';
   u_lb = [-700; -700 ;-700 ;-700 ;-700]/1000;
   u_ub =  [700 ;700 ;700 ;700 ;700]/1000;
   
   if model.Wtrack ==0
       u_lb = [-100; -60 ;-50 ;-20 ;-20]/1000;
       u_ub =  [100 ;60;150 ;10 ;20]/1000;
       
   end
       
  
   if isfield(model,'task')   
       p_lb = 1;
       p_ub = 1.4;
%        p_lb = 1;
%        p_ub = 1;
   end
   
   % make bounds for the trajectory optimization problem
   if isfield(model,'task')
       
       X_lb = [repmat([x_lb ; u_lb], N, 1);p_lb];
       X_ub = [repmat([x_ub ; u_ub], N, 1);p_ub];
       X_lb(1) = model.task.Lmin;
       X_ub(1) = model.task.Lmin;
   else
       X_lb = [repmat([x_lb ; u_lb], N, 1)];
       X_ub = [repmat([x_ub ; u_ub], N, 1)];
   end
   
   
   
   % all variables, except flywheel position, must be periodic
   model.iper = 2:model.nx;
   if isfield(model,'task')
       model.npar = numel(p_lb);
   else
       model.npar = 0;
   end
   model.nper = numel(model.iper);
%    if isfield(model,'task')
%        
%        model.ntask = problem.ntask; % sum of the task and path constaraint
% %       model.ntask = 2; % number of constraints for task when doing predictive simulation (cable length) Lmin & Lmax & Xwrist for all model.N-1 nodes
%    else
%        model.ntask = 0;
%    end
   
   %model.ntask = 3; % number of constraints for task when doing predictive simulation (intial and final cable length +
   %flywheel postion should be equal to the max cable length at tmax + intial flywheel postion should be equal to the intial cable length )
   
   % collocation grid and unknowns
   model.Nvarpernode = model.nx + model.nu  ;			% number of unknowns per node
   model.Nconpernode = model.nx ;                       % number of constraint equations per node (dynamic constraints)
   model.Nvar    = N * model.Nvarpernode + model.npar ;             % number of unknowns
   model.ncondyn = (N-1)* model.Nconpernode;
   % task constraints included periodicity and cable length and path constraints
   model.ncontaskper = model.nu + model.nper; % task : periodicty contraints
   if isfield(problem,'task')
       model.cablecnst = problem.cablecnst;
   else
       model.cablecnst =  0;
   end
%    model.ncontaskcbl = problem.cablecnst; % task : cable constraints 
   if isfield(problem,'task')
       model.nconpath = problem.nconpath ;
       model.ncontaskcbl = problem.cablecnst;
   else
       model.nconpath =  0;
       model.ncontaskcbl = 0;
   end
    % path constraints
   
   model.ntask =  model.ncontaskcbl + model.nconpath ; % sum of cable and path constraints
   
   model.Ncon    = model.ncondyn + model.ncontaskper + model.ntask;       % number of constrains (including state and control periodicity)
   if isfield(model,'task')
       model.ip = model.N*model.Nvarpernode + (1:model.npar); % since we only have 1 parameter, Npar = 1
   else
       model.ip = model.N*model.Nvarpernode; % since we only have 1 parameter, Npar = 1
   end
   
   
   
   % if oldresult was provided, use it as initial guess, otherwise use data as initial guess
   if isfield(problem,'initialguess')
%        load(problem.initialguess)%look at the contend of that
       if isfield(model,'task') % if optimizing the time inwhich the cable length is in its max
           if ischar(problem.initialguess)
               %adjusting the data (Simresult) based on number of collocation nodes(N=result.model.N)
               Simtemp = reshape(Simresult,18,70);
               Simtemp=Simtemp';
               htemp = T/(70-1);    % time step for direct collocation with 70 nodes
               tdctemp = (0:htemp:T)';    % time points for direct collocation with 70 nodes
               for j=1:18
                   Sim(:,j) = interp1(tdctemp,Simtemp(:,j),tdc);
               end
               Sim=Sim';
               Simresult = Sim(:);
%                

               if strcmp(problem.initialguess,'random')
                   X0 = [Simresult;1.052]+0.1*rand(model.Nvar,1);
                   %                  X0 = X_lb + (X_ub - X_lb).*rand(model.Nvar,1);
               elseif strcmp(problem.initialguess,'zero')
                   X0 = zeros(1261,1);
               elseif strcmp(problem.initialguess,'midpoint')
                   X0 = (X_lb + X_ub)/2;
               else
                   X0 = [Simresult;1.052];
                   %                  ix = 1:model.nx;
                   %                  for i = 1:model.N-1
                   %                      iy = ix(model.itrack);
                   %                      X0(iy)  = model.data(i,:)'; % node i
                   %                      ix = ix + model.Nvarpernode;
                   %                  end
               end
               
           else
               X0 = zeros(numel(X_lb),1);
           end
       else
           
           if ischar(problem.initialguess) % if tmax was not included in optimization
               if strcmp(problem.initialguess,'random')
                   X0 = 0.1*rand(model.Nvar,1);
               elseif strcmp(problem.initialguess,'zero')
                   X0 = zeros(numel(X_lb),1);
               elseif strcmp(problem.initialguess,'midpoint')
                   X0 = (X_lb + X_ub)/2;
               else
                   X0 = [Simresult];
                   
               end
               
           else
               X0 = zeros(numel(X_lb),1);
           end
       end
   end

  

   % Determine Jacobian structure and number of non-zeros
   Xr = rand(model.Nvar,1);
   %     Xr(end) = 1.0519;
   model.Jnnz = 1;
   J = conjac(Xr); %determins where the jacobian is not zoro
   model.conjacstructure = double(J ~= 0);% it compares each element in the jacobian to zero(it makes it one when its not zero, and zero when its zero)
   %and then it stores it in Matlab as a logical variable (binary
   %variable). (recording 4 at 1:00:00)
   model.Jnnz = nnz(J);
   model.conjacstructure(end-1:end,:) = 1;
   
   % Check the derivations, if N=10 was specified
   if N == 50
       checkderiv; % compare gradient and Jacobian against finite difference approximations
   end
   
   % solve the NLP with IPOPT
   funcs.objective   = @objfun ;
   funcs.gradient    = @objgrad;
   funcs.constraints = @confun ;
   funcs.jacobian    = @conjac ;
   funcs.jacobianstructure = @conjacstructure;

   options.lb  = X_lb;
   options.ub  = X_ub;
   options.cl  = zeros(model.Ncon,1);
   options.cu  = zeros(model.Ncon,1);
   %    if model.nconpath == model.N
   if model.nconpath>0
       
%        options.cu(model.ncondyn + model.ncontaskper + model.ncontaskcbl +( 1:model.nconpath) )  = 1.5*model.cgain;
%        options.cl(model.ncondyn + model.ncontaskper + model.ncontaskcbl +( 1:model.nconpath))  = 0.6*model.cgain;
       options.cu(model.ncondyn + model.ncontaskper + model.ncontaskcbl +(1) )  = 1.5*model.cgain;
       options.cl(model.ncondyn + model.ncontaskper + model.ncontaskcbl +( 1))  = 0.78*model.cgain;
       if model.nconpath==2          
           options.cu(model.ncondyn + model.ncontaskper + model.ncontaskcbl +(2) )  = 1.5*model.cgain;
           options.cl(model.ncondyn + model.ncontaskper + model.ncontaskcbl +( 2))  = 0.78*model.cgain;
       end
       %        options.cu(model.ncondyn + model.ncontaskper + model.ncontaskcbl +( 1:model.nconpath) )  = model.cgain*[0.984587124166634,0.949239382211693,0.940856786460101,0.945733510058762,0.957084562150634,0.972721713834140,0.982509343643580,0.987816947672726,0.990539007860171,0.990227649806674,0.994967362720877,1.00263477046094,1.00334222046470,1.00144009367118,0.997344919254536,0.989714675158803,0.986123385439148,0.977004790375032,0.968045821421840,0.958646052162770,0.948193316336059,0.941418405647705,0.938910094583808,0.924573210865546,0.906861858588330,0.892478320365616,0.881124752664981,0.912010855669652,0.931751477873481,0.944534048076951,0.949628794333322,0.949838966830169,0.948059476679608,0.945912702743256,0.939938348258617,0.930775839376757,0.923109611948986,0.912706410950738,0.907525287589332,0.913178747383131,0.918094181330246,0.925448288060352,0.931908885377663,0.938841859848728,0.947657130474933,0.958560954984599,0.970764124781298,0.984569343535761,0.991153597130201,0.999707630343451,1.00771863848097,1.01451724929020,1.01957971015749,1.02292890754400,1.02408734776336,1.02335939940512,1.02101448675558,1.01703412767700,1.01073978538003,1.00238201602046,0.991859778375628,0.980683771690154,0.972870044715305,0.971611739892830,0.969269605275424,0.965862830792782,0.961891542922626,0.952998931433828,0.948379405406072,0.984587124166634]';
       %        options.cl(model.ncondyn + model.ncontaskper + model.ncontaskcbl +( 1:model.nconpath) )  = model.cgain*[0.984587124166634,0.949239382211693,0.940856786460101,0.945733510058762,0.957084562150634,0.972721713834140,0.982509343643580,0.987816947672726,0.990539007860171,0.990227649806674,0.994967362720877,1.00263477046094,1.00334222046470,1.00144009367118,0.997344919254536,0.989714675158803,0.986123385439148,0.977004790375032,0.968045821421840,0.958646052162770,0.948193316336059,0.941418405647705,0.938910094583808,0.924573210865546,0.906861858588330,0.892478320365616,0.881124752664981,0.912010855669652,0.931751477873481,0.944534048076951,0.949628794333322,0.949838966830169,0.948059476679608,0.945912702743256,0.939938348258617,0.930775839376757,0.923109611948986,0.912706410950738,0.907525287589332,0.913178747383131,0.918094181330246,0.925448288060352,0.931908885377663,0.938841859848728,0.947657130474933,0.958560954984599,0.970764124781298,0.984569343535761,0.991153597130201,0.999707630343451,1.00771863848097,1.01451724929020,1.01957971015749,1.02292890754400,1.02408734776336,1.02335939940512,1.02101448675558,1.01703412767700,1.01073978538003,1.00238201602046,0.991859778375628,0.980683771690154,0.972870044715305,0.971611739892830,0.969269605275424,0.965862830792782,0.961891542922626,0.952998931433828,0.948379405406072,0.984587124166634]';
       %
       %917-986
   end
   
%    model.cl=options.cl ;
%    model.cu=options.cu;

   %    options.cl(end-model.ntask+2)  = 10;
   %    options.cu(end-model.ntask+2)  = 0;

   options.ipopt.max_iter = 5000;
%    options.ipopt.hessian_approximation = 'limited-memory';
   options.ipopt.hessian_approximation = 'limited-memory';

options.ipopt.tol = 1e-3;
   % options.ipopt.print_level = 0;
   
   [X, info] = ipopt(X0,funcs,options);
   
   if info.status ~= 0
       warning('IPOPT did not solve');
   end
   
   % create the structure containing the results (using the same result structure from the previous optimization)
   result.model = model;
   result.X = X;
   result.info = info;
    
    

    
end	
%====================================================================
function [f] = objfun(X)
    global model
    
    %f1 is tracking cost
    f1 = 0;
    
    if (model.Wtrack ~= 0) %same change in objgrad
        ix = 1:model.nx;
        for i = 1:model.N-1
            iy = ix(model.itrack);
            y  = X(iy);
            ymeas  = model.data(i,:)';% node i
            f1 = f1 + mean(((y - ymeas)./model.dataSD').^2);  % y has 7 variables. this is the best way to write the cost function
            ix = ix + model.Nvarpernode;
        end
        f1 = f1/(model.N-1);
    end
    
    f2 = 0;
    iu = model.nx + (1:model.nu);
%      umax = [5,5,5,0.4,0.2]';
%      umax = [1,1,1,0.4,0.2]';
%       umax = [1,1,2,0.4,0.6]';

%      umax = [1,1,1,1,1]';
%      umax = [2.89,1.18,6.97,1.09,0.57]';

    for i = 1:model.N-1
        u = X(iu);
        f2 = f2 + mean((u./model.umax).^2)*1000;        
        iu = iu + model.Nvarpernode;
    end
%     iw = 8:12;
%       for i = 1:model.N-1
%         u = X(iu);
%         w = X(iw);
%         f2 = f2 + mean((u.*w).^2);        %power
%         iu = iu + model.Nvarpernode;
%         iw = iw + model.Nvarpernode;
%       end
    f2 = f2/(model.N-1);
    % if we wanna do predictive (only have effort) we make the first weight (tracking)=0 
    f = model.Wtrack * f1 + model.Weffort * f2;

    % a short pause to make sure that IPOPT screen output appears
    % continuously (if print_level was set accordingly)
    pause(1e-6);	
end
%====================================================================
function [g] = objgrad(X)
    global model


    % initialize gradient
    g = zeros(size(X));

    % gradient of tracking cost
    ix = 1:model.nx;
%     umax = [5,5,3,0.4,0.2]';
%     umax = [1,1,1,1,1]';
%      umax = [2.89,1.18,6.97,1.09,0.57]';
%      umax = [5,5,5,0.4,0.2]';
%                umax = [1,1,1,0.4,0.2]';
%                umax = [1,1,2,0.4,0.6]';


    for i = 1:model.N-1
        iy = ix(model.itrack)';
        y  = X(iy);
        ymeas  = model.data(i,:)';
        % f1 = f1 + mean(((y - ymeas)./model.dataSD).^2); 
        g(iy) = g(iy) + model.Wtrack * 2*(y-ymeas)./(model.dataSD'.^2);
        %g(iy) = g(iy) + model.Wtrack * 2*(y-ymeas);

        g(iy) = g(iy)/(model.N-1)/model.ntrack;
        %g(iy) = g(iy)/model.ntrack;

        ix = ix + model.Nvarpernode;
    end
    
        iu = model.nx + (1:model.nu);
        for i = 1:model.N-1
            u = X(iu);
            % f2 = f2 + mean(u.^2); 
            g(iu) = g(iu) + model.Weffort * 2*u*1000./(model.umax.^2) ;
            g(iu)=g(iu)/(model.N-1)/model.nu;
            %g(iu)=g(iu)/model.nu;
            iu = iu + model.Nvarpernode;
        end
        
%         iw = 8:12;
%            for i = 1:model.N-1
%             u = X(iu);
%             w = X(iw);
%             %f2 = f2 + mean(u.*w);        
%             g(iu) = g(iu) + model.Weffort * (w );
%             g(iu)=g(iu)/(model.N-1)/model.nu;
%             g(iw)=g(iw) + model.Weffort * (u );
%             g(iw)=g(iw)/(model.N-1)/model.nu;
%             %g(iu)=g(iu)/model.nu;
%             iu = iu + model.Nvarpernode;
%             iw = iw + model.Nvarpernode;
%            end
      
%         iw = 8:12;
%            for i = 1:model.N-1
%             u = X(iu);
%             w = X(iw);
%             %f2 = f2 + (u.*w).^2;       
%             g(iu) = g(iu) + model.Weffort * (2*u.*(w.^2) );
%             g(iu)=g(iu)/(model.N-1)/model.nu;
%             g(iw)=g(iw) + model.Weffort * (2*w.*(u.^2)  );
%             g(iw)=g(iw)/(model.N-1)/model.nu;
%             %g(iu)=g(iu)/model.nu;
%             iu = iu + model.Nvarpernode;
%             iw = iw + model.Nvarpernode;
%            end

    % a short pause to make sure that IPOPT screen output appears
    % continuously (if print_level was set accordingly)
    pause(1e-6);	
end
%====================================================================
	function J = conjacstructure(X)
    
    global model
     J = model.conjacstructure;
			
    end
%====================================================================

%====================================================================
function [c] = confun(X)
    
	global model
    
    % the difference is we have to have seperate x and u
	h = model.h;
    load('Cons.mat')
    c = zeros(model.Ncon,1);     % initialize the constraints
   % c = a;     % initialize the constraints

    if  model.nconpath == 2
%         c = [a;a(end)];
%         c(model.ncondyn + model.ncontaskper + model.ncontaskcbl+1)  = 0.8014;
        c(model.ncondyn + model.ncontaskper + model.ncontaskcbl+2)  = 0.7866;
    end
    
    par.Yankle = 0.5182;
    par.ForearmLen = 0.2355;
    par.ShankLen = 0.4252;
    par.ThighLen = 0.4009;
    par.UpparmLen = 0.2856;
    par.TrunkLen = 0.4312;
    
    % dynamics constraints
    ix1 = 1:model.nx;               % index for the state variables of node 1
    iu1 = model.nx + (1:model.nu);  % index for the controls in node 1
    ic  = 1:model.Nconpernode;      % index for constraints from node 1
    p = X(model.ip);             % extract the model parameters from X
    
    for i = 1:model.N-1
        % extract variables from successive nodes
        x1 = X(ix1);
        x2 = X(ix1 + model.Nvarpernode);  
        u1 = X(iu1);
        u2 = X(iu1 + model.Nvarpernode);
        p = X(model.ip);
      
    	% dynamics constraints are calculated by dynfun function
                

        % use Backward Euler formula or Midpoint Euler formula as dynamics constraint
        if strcmp(model.discretization, 'BE')
            c(ic) = dynfun(x2 , (x2-x1)/h, u2)/model.dyngain;
        else
            c(ic) = dynfun((x1+x2)/2 , (x2-x1)/h, (u1+u2)/2)/model.dyngain;
        end
        
        ix1 = ix1 + model.Nvarpernode;
        iu1 = iu1 + model.Nvarpernode;
        ic  = ic  + model.Nconpernode;

    end

    % periodicity constraints for states and controls
    % dynamics constraints: state periodicity (all variables axcept state 1)and conbtrol periodicity
    ifirst = [model.iper model.nx+(1:model.nu)];
    ilast = (model.N-1)*model.Nvarpernode + ifirst;
    npercon = numel(ifirst);
  
    c(model.Nconpernode*(model.N-1)+(1:npercon)) = ...
        X(ifirst) - X(ilast); % node one - the last one 989-914
  
    if isfield(model,'task')
        %  task constraint for the min cable length
        x0 = X(1:model.nx);
        [~,~,~,~,L0] = dynfun(x0,zeros(model.nx,1),zeros(model.nu,1)); % the cable length L only depends the body positions (q)
        c(model.ncondyn + model.ncontaskper + 1) = L0 - model.task.Lmin;   % difference between final and initial cable length c(model.Ncon-1)
        
        %  task constraint for the max cable length
        tmax = X(end-model.npar+1);
        i = find(min(diff(tmax > model.time)) == diff(tmax > model.time));	% find the index of Model.time which is just less than tmax: Huawei
        f = (tmax-model.time(i))/(model.time(i+1)-model.time(i)); %
        xi1 =  X( (model.nx+model.nu)*(i-1) + (1:model.nx) ) ;		% the state x at node i
        [~,~,~,~,Li1] = dynfun(xi1,zeros(model.nx,1),zeros(model.nu,1)); % find the max cable length first point
        xi2 =  X( (model.nx+model.nu)*i + (1:model.nx)) ;   % the state x at node i+1
        [~,~,~,~,Li2] = dynfun(xi2,zeros(model.nx,1),zeros(model.nu,1)); % find the max cable length second point
<<<<<<< HEAD
        
        Ltmax = (1-f)*Li1 + f*Li2 ;    % linear interpolation of two above nodes
        c(model.ncondyn + model.ncontaskper + model.ncontaskcbl) = Ltmax - model.task.Lmax;   % difference between maximum cable length from the previous simulation results and maximum cable length
        if model.nconpath>0
%             for i = 1:model.N
%             for i = 1:model.nconpath
            for i = 38
                
                q1= X((model.nx+model.nu)*(i-1)+2);
                q2= X((model.nx+model.nu)*(i-1)+3);
                q3= X((model.nx+model.nu)*(i-1)+4);
                q4= X((model.nx+model.nu)*(i-1)+5);
                q5= X((model.nx+model.nu)*(i-1)+6);
%                 q = [q1,q2,q3,q4,q5]';%trasnpose
%                 %
%                 qddot = zeros(5,1);
%                 qdot = zeros(5,1);
%                 
%                 %           F(:,i) = X((model.nx+model.nu)*(i-1)+13);%zero for this also
%                 F = 0;
%                 
%                 [~,~,~,~,~,~,~,~,stick] = rowerdynamics(q,qdot,qddot,F,model.parameters);
%                 wrist(i) = stick (6,2)';%keep the wrist higher than a constant value(so we dont need to use hip)
                %             wristXmine(i) = par.Yankle - par.ShankLen*sin(-q1(i)) +par.ThighLen*sin(q2(i)+q1(i))+...
                %                 par.TrunkLen*sin(-q2(i)-q1(i)-q3(i))- par.UpparmLen*sin(-q2(i)-q1(i)-q3(i)-q4(i))+...
                %                 par.ForearmLen*sin(+q2(i)+q1(i)+q3(i)+q4(i)+q5(i));
                % calculate the vertical cordinate of the wrist
                                c(model.ncondyn + model.ncontaskper + model.ncontaskcbl+1) = (par.Yankle + par.ShankLen*sin(pi-q1) -...
                                    par.ThighLen*sin(q2-pi+q1)+...
                                    par.TrunkLen*sin(-q2-q1-q3+2*pi)- par.UpparmLen*sin(-q2-q1-q3-q4+2*pi)+...
                                    par.ForearmLen*sin(-2*pi+q2+q1+q3+q4+q5))*model.cgain;
%                                 for j=model.N-35
                
                
            end
            if model.nconpath==2
                
                for i = 38
                    
                    q1= X((model.nx+model.nu)*(i-1)+2);
                    q2= X((model.nx+model.nu)*(i-1)+3);
                    q3= X((model.nx+model.nu)*(i-1)+4);
                    q4= X((model.nx+model.nu)*(i-1)+5);
                    q5= X((model.nx+model.nu)*(i-1)+6);
                    c(model.ncondyn + model.ncontaskper + model.ncontaskcbl+2) = (par.Yankle + par.ShankLen*sin(pi-q1) -...
                        par.ThighLen*sin(q2-pi+q1)+...
                        par.TrunkLen*sin(-q2-q1-q3+2*pi)- par.UpparmLen*sin(-q2-q1-q3-q4+2*pi)+...
                        par.ForearmLen*sin(-2*pi+q2+q1+q3+q4+q5))*model.cgain;
                end
            end
        end
=======
        Ltmax = (1-f)*Li1 + f*Li2 ;    % linear interpolation of two above nodes
        c(end-model.ntask+2) = Ltmax - model.task.Lmax;   % difference between maximum cable length from the previous simulation results and maximum cable length
    end
         
%    %task constraint for the flywheel position (in case of using both costraint for flywheel position model.ntask=4
%    %or if using just one of them model.ntask=3 )
%     flywheel_pos1 = xi1(1);
%     flywheel_pos2 = xi2(1);
%     flywheel_pos = f*flywheel_pos1 + (1-f)*flywheel_pos2;
%     c(end-model.ntask+3) = flywheel_pos - model.task.Lmax; % difference between maximum cable length and flwyheel position

    
>>>>>>> e109c3f2d5672b4b3050c1e1f35ca9f7162632ec

    end
             model.c=c;
             plot(model.c)
   
%     a=c(917:model.Ncon);
% plot(c,'r');hold on;plot(model.cl,'b');hold on;plot(model.cu,'g')
end

%====================================================================
function [J] = conjac(X)
	
    global model
	h = model.h;
    c = zeros(model.Ncon,1);     % initialize the constraints
    par.Yankle = 0.5182;
    par.ForearmLen = 0.2355;
    par.ShankLen = 0.4252;
    par.ThighLen = 0.4009;
    par.UpparmLen = 0.2856;
    par.TrunkLen = 0.4312;
    

    % initialize the sparse Jacobian matrix
	J = spalloc(model.Ncon,model.Nvar, model.Jnnz);		
    
    % dynamics constraints
 	ix1 = 1:model.nx;   % index for the variables of node 1
    ic  = 1:model.Nconpernode;   % index for constraints from node 1
    iu1 = model.nx + (1:model.nu);  % index for the controls in node 1

    for i=1:model.N-1
		% extract variables from two successive nodes
		x1 = X(ix1);
		ix2 = ix1 + model.Nvarpernode;
		x2 = X(ix2);
        u1 = X(iu1);
        iu2 = iu1 + model.Nvarpernode;
        u2 = X(iu2);
        p = X(model.ip);

        if strcmp(model.discretization, 'BE')
            [~, dfdx, dfdxdot,dfdu]= dynfun(x2, (x2-x1)/h, u2);   %?????dLdq
            
            J(ic,ix1) = -dfdxdot/h/model.dyngain;
            J(ic,ix2) = (dfdx + dfdxdot/h)/model.dyngain;
            J(ic, iu2)  = dfdu/model.dyngain;
        else
            [~, dfdx, dfdxdot,dfdu] = dynfun((x1+x2)/2, (x2-x1)/h, (u1+u2)/2);    %?????dLdq
            J(ic,ix1) = (dfdx/2 - dfdxdot/h)/model.dyngain;
            J(ic,ix2) = (dfdx/2 + dfdxdot/h/model.dyngain);
            J(ic, iu1)  = dfdu/2/model.dyngain;
            J(ic, iu2)  = dfdu/2/model.dyngain;
        end

		%  advance ix1 and irow to next node
		ix1 = ix1 + model.Nvarpernode;
		ic  = ic  + model.Nconpernode;
        iu1 = iu1 + model.Nvarpernode;
      
        
    end

    % periodicity constraints for states and controls
    % state periodicity (all variables axcept state 1)and conbtrol periodicity
    ifirst = [model.iper model.nx+(1:model.nu)];
    ilast = (model.N-1)*model.Nvarpernode + ifirst;
    npercon = numel(ifirst);
   
    %[1:model.Nconpernode*(model.N-1) where we have dynamic constraints] [dynamic constraints + (1:npercon) where we have periodicity constraints]
    c(model.Nconpernode*(model.N-1)+(1:npercon)) = X(ifirst) - X(ilast); % node one - the last one 
    J(model.Nconpernode*(model.N-1)+(1:npercon) , ifirst) = speye(npercon);
    J(model.Nconpernode*(model.N-1)+(1:npercon) , ilast) = -speye(npercon);
    
    if isfield(model,'task')
        
        %  task constraint for the min cable length
        x0 = X(1:model.nx);
        [~,~,~,~,L0,dL0dq] = dynfun(x0,zeros(model.nx,1),zeros(model.nu,1)); % the cable length L only depends the body positions (q)
        %     c(end-model.ntask+1) = L0 - model.task.Lmin;   % difference between initial cable length and the min cable length from the data
        J(model.ncondyn + model.ncontaskper + 1 , 2:6 ) =  dL0dq;  % L0 is a function of qs (2-6)
        
        %  task constraint for the max cable length
        tmax = X(end-model.npar+1);
        %       i = find( min(diff(tmax > model.time)) == diff(tmax > model.time) );	% find the index of Model.time which is just less than tmax: Huawei
        i = find( -1 == diff(tmax > model.time) );	% find the index of Model.time which is just less than tmax: Huawei
        
        f = (tmax-model.time(i))/(model.time(i+1)-model.time(i)); %
        xi1 =  X( (model.nx+model.nu)*(i-1) + (1:model.nx) ) ;		% the state x at node i
        [~,~,~,~,Li1,dLi1dt] = dynfun(xi1,zeros(model.nx,1),zeros(model.nu,1)); % find the max cable length first point
        xi2 =  X( (model.nx+model.nu)*i + (1:model.nx)) ;   % the state x at node i+1
        [~,~,~,~,Li2,dLi2dt] = dynfun(xi2,zeros(model.nx,1),zeros(model.nu,1)); % find the max cable length second point
<<<<<<< HEAD
        %         Ltmax = (1-f)*Li1 + f*Li2 ;    % linear interpolation of two above nodes
        %         c(end-model.ntask+2) = Ltmax - model.task.Lmax;   % difference between maximum cable length from the previous simulation results and maximum cable length
        
        J(model.ncondyn + model.ncontaskper + model.ncontaskcbl, (model.nx+model.nu)*(i-1) + (2:6)) = (1-f)* dLi1dt ;
        J(model.ncondyn + model.ncontaskper + model.ncontaskcbl , (model.nx+model.nu)*i + (2:6)) = f*dLi2dt;
        dLtmaxdf = -Li1 + Li2;
=======
        Ltmax = (1-f)*Li1 + f*Li2 ;    % linear interpolation of two above nodes
        J(end-model.ntask+2 , (model.nx+model.nu)*(i-1) + (2:6)) = f* dLi1dt ;
        J(end-model.ntask+2 , (model.nx+model.nu)*i + (2:6)) = (1-f)*dLi2dt;
        dLtmaxdf = Li1 - Li2;
>>>>>>> e109c3f2d5672b4b3050c1e1f35ca9f7162632ec
        dfdtmax = 1/(model.time(i+1)-model.time(i));
        J(model.ncondyn + model.ncontaskper + model.ncontaskcbl, end) = dLtmaxdf*dfdtmax;
        
        
        if model.nconpath>0
       
%             for i = 1:model.nconpath
            for i = 38

                q1= X((model.nx+model.nu)*(i-1)+2);
                q2= X((model.nx+model.nu)*(i-1)+3);
                q3= X((model.nx+model.nu)*(i-1)+4);
                q4= X((model.nx+model.nu)*(i-1)+5);
                q5= X((model.nx+model.nu)*(i-1)+6);
                
                %     wristXmine(i) = par.Yankle + par.ShankLen*sin(pi-q1(i)) -par.ThighLen*sin(q2(i)-pi+q1(i))+...
                %     par.TrunkLen*sin(-q2(i)-q1(i)-q3(i)+2*pi)- par.UpparmLen*sin(-q2(i)-q1(i)-q3(i)-q4(i)+2*pi)+...
                %     par.ForearmLen*sin(-2*pi+q2(i)+q1(i)+q3(i)+q4(i)+q5(i));
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+1 , (model.nx+model.nu)*(i-1) +2 ) =  (-par.ShankLen*cos(pi-q1)-...
                    par.ThighLen*cos(q2+q1-pi)-...
                    par.TrunkLen*cos(-q2-q1-q3)+...
                    par.UpparmLen*cos(-q2-q1-q3-q4) +...
                    par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
                
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+1  , (model.nx+model.nu)*(i-1) +3 ) = (-par.ThighLen*cos(q2+q1-pi)-...
                    par.TrunkLen*cos(-q2-q1-q3)+ par.UpparmLen*cos(-q2-q1-q3-q4)+...
                    par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
                
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+1 , (model.nx+model.nu)*(i-1) +4 ) = (-par.TrunkLen*cos(-q2-q1-q3)+...
                    par.UpparmLen*cos(-q2-q1-q3-q4)+...
                    par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
                
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+1, (model.nx+model.nu)*(i-1) +5 ) = (par.UpparmLen*cos(-q2-q1-q3-q4)+...
                    par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
                
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+1, (model.nx+model.nu)*(i-1) +6 ) = (par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
            end
            if model.nconpath==2
            for i = 38

                q1= X((model.nx+model.nu)*(i-1)+2);
                q2= X((model.nx+model.nu)*(i-1)+3);
                q3= X((model.nx+model.nu)*(i-1)+4);
                q4= X((model.nx+model.nu)*(i-1)+5);
                q5= X((model.nx+model.nu)*(i-1)+6);
                
                %     wristXmine(i) = par.Yankle + par.ShankLen*sin(pi-q1(i)) -par.ThighLen*sin(q2(i)-pi+q1(i))+...
                %     par.TrunkLen*sin(-q2(i)-q1(i)-q3(i)+2*pi)- par.UpparmLen*sin(-q2(i)-q1(i)-q3(i)-q4(i)+2*pi)+...
                %     par.ForearmLen*sin(-2*pi+q2(i)+q1(i)+q3(i)+q4(i)+q5(i));
%                 model.ncondyn + model.ncontaskper + model.ncontaskcbl +( 1:i)
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+2 , (model.nx+model.nu)*(i-1) +2 ) =  (-par.ShankLen*cos(pi-q1)-...
                    par.ThighLen*cos(q2+q1-pi)-...
                    par.TrunkLen*cos(-q2-q1-q3)+...
                    par.UpparmLen*cos(-q2-q1-q3-q4) +...
                    par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
                
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+2  , (model.nx+model.nu)*(i-1) +3 ) = (-par.ThighLen*cos(q2+q1-pi)-...
                    par.TrunkLen*cos(-q2-q1-q3)+ par.UpparmLen*cos(-q2-q1-q3-q4)+...
                    par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
                
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+2 , (model.nx+model.nu)*(i-1) +4 ) = (-par.TrunkLen*cos(-q2-q1-q3)+...
                    par.UpparmLen*cos(-q2-q1-q3-q4)+...
                    par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
                
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+2, (model.nx+model.nu)*(i-1) +5 ) = (par.UpparmLen*cos(-q2-q1-q3-q4)+...
                    par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
                
                J(model.ncondyn + model.ncontaskper + model.ncontaskcbl+2, (model.nx+model.nu)*(i-1) +6 ) = (par.ForearmLen*cos(q2+q1+q3+q4+q5))*model.cgain;
            end
            end
        end
    end
    
        


end
%====================================================================
 function checkderiv
	% using finite differences to check that the code in objgrad and conjac is correct
	global model
    
    NX   = model.Nvar;
	hh       = 1e-6;
    %X        = randn(NX,1);
    X        = rand(NX,1); % a random vector of unknowns
	f        = objfun(X);
	grad     = objgrad(X);
	c        = confun(X);
    Ncon     = size(c,1);
	cjac     = conjac(X);
    %cjac_num = zeros(Ncon, NX);
	cjac_num = spalloc(Ncon, NX,1000);

	grad_num = zeros(NX,1);

    for i=1:NX
        fprintf('checking derivatives for unknown %4d of %4d\n',i,NX);
        Xisave        = X(i);
        X(i)          = X(i) + hh;
        cjac_num(:,i) = (confun(X) - c)/hh;
        grad_num(i)   = (objfun(X) - f)/hh;
        X(i)          = Xisave;
    end
	
	% report maximal differences between analytical derivatives and numerical results
	fprintf('Max. error in constraint jacobian: ');
	matcompare(cjac, cjac_num);
	fprintf('Max. error in objective gradient: ');
% 	matcompare(grad', grad_num);
    matcompare(grad, grad_num);

	disp('Type dbcont to continue')
	keyboard	
    end
 %====================================================================
    function matcompare(a,b)
	% compares two matrices and prints element that has greatest difference
	[maxerr,irow] = max(abs(a-b));
	[maxerr,icol] = max(maxerr);
	irow = irow(icol);
	fprintf('%9.6f at %d %d (%9.6f vs. %9.6f)\n', full(maxerr), irow, icol, full(a(irow,icol)), full(b(irow,icol)));
    end
 