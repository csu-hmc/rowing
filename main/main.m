function main
    % performs all the optimizations and produces the plots and tables
    close all
    clc
    clear global
    global result problem
    
    % Add that folder plus all subfolders to the path.
    %       mkdir('optimizer')
    addpath(genpath('optimizer'))
    addpath(genpath('model'))
    addpath(genpath('tools'))
    addpath(genpath('data'))
 
    problem.initialguess = 'sim';%pu th efilename here
    problem.cgain =1; % scaling factor for path constraint
    problem.dyngain = 1; % scaling factor for non path constraint
    problem.np =1;                 % on/off switch for constraints
    problem.N = 70;		        % number of collocation points
%     problem.Reqpower = 0.0976; % we can define the woring power here 
    %( we can either use power or path constraint-so if using problem.Reqpower make sure that problem.nconpath  = 0)
    if problem.np == 1
        problem.task.Lmin = 0.2077;       % min and max of cable lenght from the data
        problem.task.Lmax =  1.1288;
        % task constraints---> when increased even by 1 either for cable for wrist position, the IPOPT cannot solve the
        % problem anymore
        problem.cablecnst = 2; % number of constraints for task when doing predictive simulation (cable length) Lmin & Lmax
        problem.nconpath  = 1; % Zwrist constraint (depends on the number of nodes can be varies between 0 and 2 for now)
        % when increasing the number of path constraint to 2 IPOPT connot
        % solve it anymore!
    else
        problem.ntask = 0;
    end

    
    problem.component = 'spring';   % choose if we want to use damper or spring
    problem.discretization = 'BE';  % Backward Euler(BE) or Midpoint Euler(ME) discretization
    problem.tracking = 3;           % if 1 tracking the angles only else track all states (but flywheel velocity)
    % objective function gains (Wtrack=1 for the tracking  and Weffort=1 for predictive simulation) 
    problem.Wtrack =0;
    problem.Weffort =1; 
    % tracking problem:when problem.Wtrack =1 and problem.Weffort =0 works perfectly;
    % peredictive simulation problem: when problem.Wtrack =0 and problem.Weffort =1 works perfectly;
    % for now we only use predictive simulation
    result = optimize(problem);
    save 'rowingtest' result
end
