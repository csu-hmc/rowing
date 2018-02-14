function main
    % performs all the optimizations and produces the plots and tables
	close all
    clc
    clear
    global result problem 

% Add that folder plus all subfolders to the path.
%     mkdir('optimizer')
%     addpath(genpath('optimizer'))
%     addpath(genpath('model'))
%     addpath(genpath('tools'))   
%     addpath(genpath('data'))  
    

    problem.initialguess = 'sim';
	problem.N = 70;			        % number of collocation points
%     problem.initialguess = 'midpoint';
% 	problem.N = 4;			        % number of collocation points
%     model.task.Lmin = 0.2077;       % min and max of cable lenght from the data
%     model.Lmax.Lmin =  1.1288;
    problem.discretization = 'BE';  % Backward Euler(BE) or Midpoint Euler(ME) discretization
%     problem.cabletask = 1;
    problem.tracking = 3;           % if 1 tracking the angles only else track all states (but flywheel velocity)
    problem.Wtrack = 0; 
    problem.Weffort = 1;
    result = optimize(problem);
    save 'rowingtest' result
end
