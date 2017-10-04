function makeplots(filename)
    
    load(filename);
    model = result.model;
    info = result.info;
    X = result.X;

    N = model.N;
    
    nvar = 18;
    ix = 1:13;
    L = zeros(N,1);
    for i = 1:N
        x = X(ix);
        u = zeros(5,1);
        xdot = zeros(13,1);
        [~,~,~,~,L(i)] = dynfun(x,xdot,u); % the cable length L is only depends on the flywheel position
        ix = ix + nvar;
    end
    
    i1 = 1:N;	
    
    % specifying index for all states except torques
    iy1 = 18*i1-17;
    iy2 = 18*i1-16;       
    iy3 = 18*i1-15; 
    iy4= 18*i1-14; 
    iy5 = 18*i1-13; 
    iy6 = 18*i1-12; 
    iy7 = 18*i1-11; 
    iy13 = 18*i1-5;
    
    % specifying index for torques
    iu1 = 18*i1-4;
    iu2 = 18*i1-3;
    iu3 = 18*i1-2;
    iu4 = 18*i1-1;
    iu5 = 18*i1;
    
    ymeas  = model.data';% node i
    T = model.time ;

    ankle_angle = sqrt(mean(((X(iy2)' - ymeas(1,:)).^2)));
    knee_angle = sqrt(mean(((X(iy3)' - ymeas(2,:)).^2)));
    hip_angle = sqrt(mean(((X(iy4)' - ymeas(3,:)).^2)));
    shoulder_angle = sqrt(mean(((X(iy5)' - ymeas(4,:)).^2)));
    elbow_angle = sqrt(mean(((X(iy6)' - ymeas(5,:)).^2)));
%     fw_velocity = sqrt(mean(((X(iy7)' - ymeas(6,:)).^2)));
%     handle_force = sqrt(mean(((X(iy13)' - ymeas(7,:)).^2)));
%     

    fprintf('    ankle_angle error: %8.3f\n\n', ankle_angle);
    fprintf('    knee_angle error: %8.3f\n\n', knee_angle);
    fprintf('    hip_angle error: %8.3f\n\n', hip_angle);
    fprintf('    shoulder_angle error: %8.3f\n\n', shoulder_angle);
    fprintf('    elbow_angle error: %8.3f\n\n', elbow_angle);
%     fprintf('    fw_velocity error: %8.3f\n\n', fw_velocity);
%     fprintf('    handle_force error: %8.3f\n\n', handle_force);

	figure(1);clf
  	subplot(5,1,1);
	plot(T, X(iy2)','r', 'LineWidth',2);
	hold on; plot(T, ymeas(1,:),'b:' ,'LineWidth',2);
    ylabel({'ankle','pos (rad)'},'fontweight','bold','fontsize',10)
	legend('simulation','data');
    

 	subplot(5,1,2);
	plot(T, X(iy3),'r', 'LineWidth',2);
	hold on; plot(T, ymeas(2,:),'b:' ,'LineWidth',2);
    ylabel({'knee','pos (rad)'},'fontweight','bold','fontsize',10)

    

 	subplot(5,1,3);
	plot(T, X(iy4),'r', 'LineWidth',2);
	hold on; plot(T, ymeas(3,:),'b:' ,'LineWidth',2);
    ylabel({'hip','pos (rad)'},'fontweight','bold','fontsize',10)

    
 	subplot(5,1,4);
	plot(T, X(iy5),'r', 'LineWidth',2);
	hold on; plot(T, ymeas(4,:),'b:' ,'LineWidth',2);
    ylabel({'shoulder','pos (rad)'},'fontweight','bold','fontsize',10)

    

 	subplot(5,1,5);
	plot(T, X(iy6),'r', 'LineWidth',2);
	hold on; plot(T, ymeas(5,:),'b:' ,'LineWidth',2);
    ylabel({'elbow','pos (rad)'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);
    
     figure(2);
%  	subplot(3,1,1);
% 	plot(T, X(iy7),'r', 'LineWidth',2);
% 	hold on; plot(T, ymeas(6,:),'b:' ,'LineWidth',2);
%     ylabel({'fly wheel','velocity (m/s)'},'fontweight','bold','fontsize',10)
% 	legend('simulation','data');
%     
% 
%  	subplot(3,1,2);
% 	plot(T, X(iy13),'r', 'LineWidth',2);
% 	hold on; plot(T, ymeas(7,:),'b:' ,'LineWidth',2);
%     ylabel({'force','(N)'},'fontweight','bold','fontsize',10)

 	subplot(3,1,1);
	plot(T, X(iy7),'r', 'LineWidth',2);
    ylabel({'fly wheel','velocity (m/s)'},'fontweight','bold','fontsize',10)
	legend('simulation','data');
    
 	subplot(3,1,2);
	plot(T, X(iy13),'r', 'LineWidth',2);
    ylabel({'force','(N)'},'fontweight','bold','fontsize',10)

 	subplot(3,1,3);
	plot(T, X(iy1)','r', 'LineWidth',2);
    ylabel({'fly wheel','pos (rad)'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);

    
    
    figure(3);
 	subplot(5,1,1);
    plot(T, X(iu1)','b', 'LineWidth',2);
    ylabel({'ankle','torque (Nm)'},'fontweight','bold','fontsize',10)
    
    subplot(5,1,2);
    plot(T, X(iu2)','b', 'LineWidth',2);
    ylabel({'knee','torque (Nm)'},'fontweight','bold','fontsize',10)
    
    subplot(5,1,3);
    plot(T, X(iu3)','b', 'LineWidth',2);
    ylabel({'hip','torque (Nm)'},'fontweight','bold','fontsize',10)
    
    subplot(5,1,4);
    plot(T, X(iu4)','b', 'LineWidth',2);
    ylabel({'shoulder','torque (Nm)'},'fontweight','bold','fontsize',10)
    
    subplot(5,1,5);
    plot(T, X(iu5)','b', 'LineWidth',2);
    ylabel({'wrist','torque (Nm)'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);
    
      
    figure(4);
    plot(T,L,'b', 'LineWidth',2)
    ylabel({'L (m)'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);
    
    
    figure(5);
    plot(T,L-X(iy1),'b', 'LineWidth',2)
    title('Cable length')
    ylabel({'cable length (m)'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);
    
    
    q = [X(iy2),X(iy3),X(iy4),X(iy5),X(iy6)];						    % extract joint angle trajectories (states 1-5)
    figure(6);
    animate(q,model.parameters);  		
    
	% make plots
    figure(7)
    plot(T, q*180/pi, 'LineWidth',2);
    legend('q1','q2','q3','q4','q5');
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('angle (deg)','fontweight','bold','fontsize',10);
    title('Joint Angles','fontweight','bold','fontsize',10)

end

