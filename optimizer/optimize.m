
function result = optimize(problem)

% Input: problem, a structure with the following fields:
%   filename...(scalar) data file
%   N..........(scalar) number of collocation points 
%   Wtrack.....(scalar) weight of tracking term 
%   Weffort....(scalar) weight of effort term in objective
%   X..........(scalar) initial guess (default: zeros)
    
    global model 

   [dSPACEdata, qdata, t, model.parameters] = datainterp2;

    T = t(end); %duration of one rowing cycle
   
   % decode the problem
    N = problem.N;      % number of collocation points
    model.N = N;
    model.Wtrack  = problem.Wtrack; 
    model.Weffort = problem.Weffort;
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
    

    % store the tracking data in global variables
    model.time =  tdc;
    model.data = [qdata fwveldata forcedata];   % add mocap data (angles) as extra columns
 	model.dataSD = std(model.data);
    model.itrack = [2 3 4 5 6 7 13];       % the state variables to track (model.itrack=problem.irack so it should go to the script) 
    model.ntrack = numel(model.itrack);
     
    model.discretization = problem.discretization;  % Backward Euler(BE) or Midpoint Euler(ME) discretization
    model.N = problem.N;
    model.Wtrack  = problem.Wtrack; 
    model.Weffort = problem.Weffort;
    
    
    % define the state variables x, controls u, and their bounds
    % x = [x_fw, q1 ...q5, v_fw, qd1...qd5, F(kn)]
    % u = [M1(knm)...M5];
    model.nx = 13;
    model.nu = 5;

    x_lb = [0    0     -2*pi    0   -pi/2       0   0  -10 -10 -10 -10 -10  0]';
    x_ub = [10  2*pi   2*pi  2*pi   pi/3      2*pi  2  20  20  20  20  20  500 ]';
    u_lb = [-700; -700 ;-700 ;-700 ;-700];
    u_ub =  [700 ;700 ;700 ;700 ;700];
    % make bounds for the trajectory optimization problem
    X_lb = [repmat([x_lb ; u_lb], N, 1)];
    X_ub = [repmat([x_ub ; u_ub], N, 1)];
    
    % all variables, except flywheel position, must be periodic 
    model.iper = 2:model.nx;  		
    model.nper = numel(model.iper);

   	% collocation grid and unknowns
    model.Nvarpernode = model.nx + model.nu;			% number of unknowns per node
    model.Nconpernode = model.nx;                       % number of constraint equations per node (number of dynamic equations)
    model.Nvar    = N * model.Nvarpernode ;             % number of unknowns 
	model.Ncon    = (N-1)* model.Nconpernode + model.nper + model.nu ;       % number of constrains (including state and control periodicity)
    % for turning the muscles on and off we can do it by going in a model
    % and say number of muscles is zero

    % if oldresult was provided, use it as initial guess, otherwise use data as initial guess
    if isfield(problem,'initialguess')
        if (numel(problem.initialguess) == model.Nvar)
            X0 = problem.initialguess.X;
        else
            error('initial guess did not have the same N');
            % resampling to N nodes, to be written
        end
    else
          X0 = (X_lb + X_ub)/2;
%         X0 = zeros(numel(X_lb),1);

        ix = 1:model.nx;
        for i = 1:model.N-1
          iy = ix(model.itrack);
          X0(iy)  = model.data(i,:)'; % node i
          ix = ix + model.Nvarpernode;
        end
%         iy = ix(model.itrack);
%         X0(iy)  = model.data(i,:)'; % node i          
    end

  

    % Determine Jacobian structure and number of non-zeros
    Xr = rand(model.Nvar,1);
    model.Jnnz = 1;
    J = conjac(Xr); %determins where the jacobian is not zoro
    model.conjacstructure = double(J ~= 0);% it compares each element in the jacobian to zero(it makes it one when its not zero, and zero when its zero)  
    %and then it stores it in Matlab as a logical variable (binary
    %variable). (recording 4 at 1:00:00)
    model.Jnnz = nnz(J);

    % Check the derivations, if N=10 was specified
    if N == 4
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
    options.ipopt.max_iter = 5000;
    options.ipopt.hessian_approximation = 'limited-memory';
    options.ipopt.tol = 1e-3;
	% options.ipopt.print_level = 0;
    
    [X, info] = ipopt(X0,funcs,options);
    
	if info.status ~= 0
		error('IPOPT did not solve');
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
    ix = 1:model.nx;
    for i = 1:model.N-1
        iy = ix(model.itrack);
        y  = X(iy);
        ymeas  = model.data(i,:)';% node i
        f1 = f1 + mean(((y - ymeas)./model.dataSD').^2);  % y has 7 variables. this is the best way to write the cost function      
%         f1 = f1 + mean(((y - ymeas)).^2);  % y has 7 variables. this is the best way to write the cost function      

        ix = ix + model.Nvarpernode;
    end
    f1 = f1/(model.N-1);
    
    
    f2 = 0;
    iu = model.nx + (1:model.nu);
    for i = 1:model.N-1
        u = X(iu);
        f2 = f2 + mean(u.^2);        
        iu = iu + model.Nvarpernode;
    end
    f2 = f2/(model.N-1);

     % using two for loops make it more readable
     % if we wanna do predictive (only have effort) we make the first weight (tracking)=0 
%     f = model.Wtrack * (f1(1)+f1(2)+f1(3)+f1(4)+f1(5)+f1(6)+f1(7))/model.ntrack + model.Weffort * f2;
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
    for i = 1:model.N-1
        iy = ix(model.itrack)';
        y  = X(iy);
        ymeas  = model.data(i,:)';
        % f1 = f1 + mean(((y - ymeas)./model.dataSD).^2); 
        g(iy) = g(iy) + model.Wtrack * 2*(y-ymeas)./(model.dataSD'.^2);
%         g(iy) = g(iy) + model.Wtrack * 2*(y-ymeas);

         g(iy) = g(iy)/(model.N-1)/model.ntrack;
 %       g(iy) = g(iy)/model.ntrack;

        ix = ix + model.Nvarpernode;
    end


        iu = model.nx + (1:model.nu);
        for i = 1:model.N-1
            u = X(iu);
            % f2 = f2 + mean(u.^2); 
            g(iu) = g(iu) + model.Weffort * 2*u;
            g(iu)=g(iu)/(model.N-1)/model.nu;
     %       g(iu)=g(iu)/model.nu;
    
            iu = iu + model.Nvarpernode;
        end

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
    c = zeros(model.Ncon,1);     % initialize the constraints
    
    % dynamics constraints
    ix1 = 1:model.nx;               % index for the state variables of node 1
    iu1 = model.nx + (1:model.nu);  % index for the controls in node 1
    ic  = 1:model.Nconpernode;      % index for constraints from node 1
    
    for i = 1:model.N-1
        % extract variables from successive nodes
        x1 = X(ix1);
        x2 = X(ix1 + model.Nvarpernode);  
        u1 = X(iu1);
        u2 = X(iu1 + model.Nvarpernode);
      
    	% dynamics constraints are calculated by dynfun function
                

        % use Backward Euler formula or Midpoint Euler formula as dynamics constraint
        if strcmp(model.discretization, 'BE')
            c(ic) = dynfun(x2 , (x2-x1)/h, u2);
        else
            c(ic) = dynfun((x1+x2)/2 , (x2-x1)/h, (u1+u2)/2);
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
        X(ifirst) - X(ilast); % node one - the last one
   
end

%====================================================================
function [J] = conjac( X)
	
    global model

	h = model.h;
    c = zeros(model.Ncon,1);     % initialize the constraints

    % initialize the sparse Jacobian matrix
	J = spalloc(model.Ncon,model.Nvar, model.Jnnz);		
    
    % dynamics constraints
 	ix1 = 1:model.nx;   % index for the variables of node 1
    ic  = 1:model.Nconpernode;   % index for constraints from node 1
    iu1 = model.nx + (1:model.nu);  % index for the controls in node 1
    Len = zeros(1,model.N);
    for i=1:model.N-1
		% extract variables from two successive nodes
		x1 = X(ix1);
		ix2 = ix1 + model.Nvarpernode;
		x2 = X(ix2);
        u1 = X(iu1);
        iu2 = iu1 + model.Nvarpernode;
        u2 = X(iu2);

        if strcmp(model.discretization, 'BE')
            [~, dfdx, dfdxdot,dfdu] = dynfun(x2, (x2-x1)/h, u2);
            J(ic,ix1) = -dfdxdot/h;
            J(ic,ix2) = dfdx + dfdxdot/h;
            J(ic, iu2)  = dfdu;
        else
            [~, dfdx, dfdxdot,dfdu] = dynfun((x1+x2)/2, (x2-x1)/h, (u1+u2)/2);
            J(ic,ix1) = dfdx/2 - dfdxdot/h;
            J(ic,ix2) = dfdx/2 + dfdxdot/h;
            J(ic, iu1)  = dfdu/2;
            J(ic, iu2)  = dfdu/2;
        end
        
		%  advance ix1 and irow to next node
		ix1 = ix1 + model.Nvarpernode;
		ic  = ic  + model.Nconpernode;
        iu1 = iu1 + model.Nvarpernode;
      
        
    end

    % periodicity constraints for states and controls
    % dynamics constraints: state periodicity (all variables axcept state 1)and conbtrol periodicity
    ifirst = [model.iper model.nx+(1:model.nu)];
    ilast = (model.N-1)*model.Nvarpernode + ifirst;
    npercon = numel(ifirst);
    
%     c(model.Nconpernode*(model.N-1)+(1:npercon)) = X(ifirst) - X(ilast); % node one - the last one
    J(model.Nconpernode*(model.N-1)+(1:npercon) , ifirst) = speye(npercon);
    J(model.Nconpernode*(model.N-1)+(1:npercon) , ilast) = -speye(npercon);
end
%====================================================================
 function checkderiv
	% using finite differences to check that the code in objgrad and conjac is correct
	global model
    
    NX   = model.Nvar;
	hh       = 1e-6;
%     X        = randn(NX,1);
    X        = rand(NX,1); % a random vector of unknowns
	f        = objfun(X);
	grad     = objgrad(X);
	c        = confun(X);
    Ncon     = size(c,1);
	cjac     = conjac(X);
% 	cjac_num = zeros(Ncon, NX);
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
 

