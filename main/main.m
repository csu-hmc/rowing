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
 
    problem.initialguess = 'sim';
    %wt=1 we=1
    %using sim:3.0867449e-002
    %using midpoint:3.0867449e-002
    %using random: 5.7851888e-002 
    %using zero:  IPOPT did not solve 
    
    %wt=1 we=10
    %using sim:6.9785323e-002
    %using midpoint:6.9785323e-002 
    %using random: 1.2850537e-001
    %using zero:  IPOPT did not solve 
    
    problem.N = 70;			        % number of collocation points
%   	problem.N = 4;			        % used for derivetive checking
    problem.np = 1;
    problem.task.Lmin = 0.2077;       % min and max of cable lenght from the data
    problem.task.Lmax =  1.1288;
    problem.component = 'damper'; % choose if we want to use damper or spring
    problem.discretization = 'BE';  % Backward Euler(BE) or Midpoint Euler(ME) discretization
    problem.tracking = 3;           % if 1 tracking the angles only else track all states (but flywheel velocity)
    problem.Wtrack = 1;
    problem.Weffort = 0;
    result = optimize(problem);
    save 'rowingtest' result
end
