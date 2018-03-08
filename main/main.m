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
 
    problem.initialguess = 'midpoint';
    problem.N = 70;			        % number of collocation points
    % 	problem.N = 4;			        % used for derivetive checking
    problem.np = 1;
    problem.task.Lmin = 0.2077;       % min and max of cable lenght from the data
    problem.task.Lmax =  1.1288;
    problem.component = 'damper'; % choose if we want to use damper or spring
    problem.discretization = 'BE';  % Backward Euler(BE) or Midpoint Euler(ME) discretization
    problem.tracking = 3;           % if 1 tracking the angles only else track all states (but flywheel velocity)
    problem.Wtrack = 0;
    problem.Weffort = 1;
    result = optimize(problem);
    save 'rowingtest' result
end
