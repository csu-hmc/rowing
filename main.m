function main
    % performs all the optimizations and produces the plots and tables
	close all
    clc
    clear
    global result problem 
	problem.N = 70;			        % number of collocation points
%     model.task.Lmin = 0.2077;       % min and max of cable lenght from the data
%     model.Lmax.Lmin =  1.1288;
    problem.discretization = 'BE';  % Backward Euler(BE) or Midpoint Euler(ME) discretization
    problem.cabletask = 1;
    problem.tracking = 3;           % if 1 tracking the angles only else track all states (but flywheel velocity)
    problem.Wtrack = 0; 
    problem.Weffort = 60;
    result = optimize(problem);
    save 'rowingtest' result
end
