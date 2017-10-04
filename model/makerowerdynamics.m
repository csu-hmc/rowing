%======================================================================
% this file contains Matlab code to generate rowerdynamics.m
% from the Autolev source code in rowerdynamics.al
%======================================================================
function makerowerdynamics

	% Run Autolev to generate tmp.m from rowerdynamics.al
    disp('Autolev is generating tmp.m...');
    fprintf('Hit CTRL-C if this does not complete within 1 minute\n');
    system(['"C:\Program Files\Autolev\al.exe" rowerdynamics.al > nul']);		% double quotes needed because autolev has spaces in its path
    % check if Autolev was successful
    if ~exist('tmp.m')
        error('Autolev error');
	else
		disp('Done.');
    end
	
	% Transform the matlab code tmp.m into rowerdynamics.m
    disp('Generating rowerdynamics.m...');
	NDOF = 5;

	% read the matlab code that came from Autolev
	fid1 = fopen('tmp.m','r');
	if (fid1 == -1)
		error('Could not open tmp.m');
	end
	
	% open the file where the clean matlab code is written
	fid2 = fopen('rowerdynamics.m','w');
	if (fid2 == -1)
		error('Could not write rowerdynamics.m', name);
	end
	
	% write function header
	fprintf(fid2,'%% This file was generated by makerowerdynamics.m on %s\n', datetime);
	fprintf(fid2,'function [f, df_dq, df_dqd, df_dqdd, df_dxfw, stick] = rowerdynamics(q,qd,qdd,xfw,par);\n');
    fprintf(fid2,'%%\n');
    fprintf(fid2,'%% Input:\n');
    fprintf(fid2,'%%      q...................(5x1) generalized coordinates (rad)\n');
    fprintf(fid2,'%%      qd..................(5x1) generalized velocities (rad/s)\n');
    fprintf(fid2,'%%      qdd.................(5x1) generalized accelerations (rad/s^2)\n');
    fprintf(fid2,'%%      xfw.................(scalar) flywheel position (m)\n');
    fprintf(fid2,'%%      par.................(structure) model parameters:\n');
    fprintf(fid2,'%%            par.TrunkMass,   par.TrunkInertia,   par.TrunkCM,   par.TrunkLen\n');
    fprintf(fid2,'%%            par.ThighMass,   par.ThighInertia,   par.ThighCM,   par.ThighLen\n');
    fprintf(fid2,'%%            par.ShankMass,   par.ShankInertia,   par.ShankCM,   par.ShankLen\n');
    fprintf(fid2,'%%            par.UpparmMass,  par.UpparmInertia,  par.UpparmCM,  par.UpparmLen\n');
    fprintf(fid2,'%%            par.ForearmMass, par.ForearmInertia, par.ForearmCM, par.ForearmLen\n');
    fprintf(fid2,'%%            par.Xankle, par.Yankle\n');
    fprintf(fid2,'%%            par.Xsprocket, par.Ysprocket		%% point where the cable comes out of the machine\n');
    fprintf(fid2,'%%            par.a, par.b						%% seat path is Y = a*X + b\n');
    fprintf(fid2,'%%            par.K, par.C						%% seat stiffness and damping\n');
    fprintf(fid2,'%%            par.Kcrm						    %% cable stiffness parameter	\n');
    fprintf(fid2,'%%\n');
    fprintf(fid2,'%% Output:\n');
    fprintf(fid2,'%%      f...................(6x1) 5 joint torques (Nm) and cable force (N)\n');
    fprintf(fid2,'%%      df_dq...............(6x5) Jacobian matrix df/dq\n');
    fprintf(fid2,'%%      df_dqd..............(6x5) Jacobian matrix df/dqdot\n');
    fprintf(fid2,'%%      df_dqdd.............(6x5) Jacobian matrix df/dqdotdot\n');
    fprintf(fid2,'%%      df_dxfw.............(6x1) Jacobian matrix df/dxfw\n');
    fprintf(fid2,'%%      stick...............(7x2) x,y of ankle,knee,hip,shoulder,elbow,wrist,sprocket\n');
    fprintf(fid2,'%%\n');

	% generate code to copy q, qd, qdd into scalar variables
	for i = 1:NDOF
		fprintf(fid2,'    q%1d   = q(%1d);  \n', i,i);		
		fprintf(fid2,'    q%1dp  = qd(%1d); \n', i,i);	
		fprintf(fid2,'    q%1dpp = qdd(%1d);\n', i,i);	
    end
    
    % make sure that f comes out as a 6x1 column vector
    fprintf(fid2,'    f = zeros(6,1); \n');
    
	% copy the necessary parts of C code from fid1 to fid2
	copying = 0;
	while ~feof(fid1)
		line = fgetl(fid1);
		if strncmp(line, '% Unit conversions', 18)
			copying = 1;
        elseif strncmp(line, 'function DoCalculations', 23) 	% z code ends here
			copying = 0;
        elseif strncmp(line, 'f(1) =', 6)   % encoded variables code starts here
			copying = 1;
        elseif strcmp(line, 'Encode(1) = 0.0;') 								   % and stops here
			copying = 0;
        end
        if copying
			line = strrep(line, 'par__', 'par.');			% change par__ into par.
			fprintf(fid2,'    %s\n',line);
		end
	end
	
	% close the input file
	fclose(fid1);
	
	% finish the function code
	fprintf(fid2,'    df_dq = sparse(df_dq);\n');
	fprintf(fid2,'    df_dqd = sparse(df_dqd);\n');
	fprintf(fid2,'    df_dqdd = sparse(df_dqdd);\n');
	fprintf(fid2,'    df_dxfw = sparse(df_dxfw);\n');
	fprintf(fid2,'\nend \n');
	fclose(fid2);

	% clean up
	delete('tmp.m');
	
    % Completion message
    disp('The function rowerdynamics.m is ready to use.');

end