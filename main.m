function main
% performs all the optimizations and produces the plots and tables
	close all
    global result problem
    
	problem.N = 70;			% number of collocation points
    problem.discretization = 'BE';  % Backward Euler(BE) or Midpoint Euler(ME) discretization
    problem.Wtrack = 1; 
    problem.Weffort = 0;
    result = optimize(problem);
    save 'rowingtest' result
end
