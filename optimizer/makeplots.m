function makeplots(filename)
    global model
    
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


    if model.tracking == 1
        ankle_angle = sqrt(mean(((X(iy2)' - ymeas(1,:)).^2)));
        knee_angle = sqrt(mean(((X(iy3)' - ymeas(2,:)).^2)));
        hip_angle = sqrt(mean(((X(iy4)' - ymeas(3,:)).^2)));
        shoulder_angle = sqrt(mean(((X(iy5)' - ymeas(4,:)).^2)));
        elbow_angle = sqrt(mean(((X(iy6)' - ymeas(5,:)).^2)));
    elseif model.tracking == 2
        ankle_angle = sqrt(mean(((X(iy2)' - ymeas(1,:)).^2)));
        knee_angle = sqrt(mean(((X(iy3)' - ymeas(2,:)).^2)));
        hip_angle = sqrt(mean(((X(iy4)' - ymeas(3,:)).^2)));
        shoulder_angle = sqrt(mean(((X(iy5)' - ymeas(4,:)).^2)));
        elbow_angle = sqrt(mean(((X(iy6)' - ymeas(5,:)).^2)));
        handle_force = sqrt(mean(((X(iy13)' - ymeas(6,:)).^2)));        
    elseif model.tracking == 3
        ankle_angle = sqrt(mean(((X(iy2)' - ymeas(1,:)).^2)));
        knee_angle = sqrt(mean(((X(iy3)' - ymeas(2,:)).^2)));
        hip_angle = sqrt(mean(((X(iy4)' - ymeas(3,:)).^2)));
        shoulder_angle = sqrt(mean(((X(iy5)' - ymeas(4,:)).^2)));
        elbow_angle = sqrt(mean(((X(iy6)' - ymeas(5,:)).^2)));
        fw_velocity = sqrt(mean(((X(iy7)' - ymeas(6,:)).^2)));
        handle_force = sqrt(mean(((X(iy13)' - ymeas(7,:)).^2)));
        
    elseif model.tracking == 4
        shoulder_angle = sqrt(mean(((X(iy5)' - ymeas(1,:)).^2)));   
    else
        shoulder_angle = sqrt(mean(((X(iy5)' - ymeas(1,:)).^2)));   
        elbow_angle = sqrt(mean(((X(iy6)' - ymeas(2,:)).^2)));
        
    end


    if model.tracking == 1
        fprintf('    ankle_angle error: %8.3f\n\n', ankle_angle);
        fprintf('    knee_angle error: %8.3f\n\n', knee_angle);
        fprintf('    hip_angle error: %8.3f\n\n', hip_angle);
        fprintf('    shoulder_angle error: %8.3f\n\n', shoulder_angle);
        fprintf('    elbow_angle error: %8.3f\n\n', elbow_angle);
    elseif model.tracking == 2
        fprintf('    ankle_angle error: %8.3f\n\n', ankle_angle);
        fprintf('    knee_angle error: %8.3f\n\n', knee_angle);
        fprintf('    hip_angle error: %8.3f\n\n', hip_angle);
        fprintf('    shoulder_angle error: %8.3f\n\n', shoulder_angle);
        fprintf('    elbow_angle error: %8.3f\n\n', elbow_angle);
        fprintf('    handle_force error: %8.3f\n\n', handle_force);
        
    elseif model.tracking == 3
        fprintf('    ankle_angle error: %8.3f\n\n', ankle_angle);
        fprintf('    knee_angle error: %8.3f\n\n', knee_angle);
        fprintf('    hip_angle error: %8.3f\n\n', hip_angle);
        fprintf('    shoulder_angle error: %8.3f\n\n', shoulder_angle);
        fprintf('    elbow_angle error: %8.3f\n\n', elbow_angle);
        fprintf('    fw_velocity error: %8.3f\n\n', fw_velocity);
        fprintf('    handle_force error: %8.3f\n\n', handle_force);
        
    elseif model.tracking == 4
        fprintf('    shoulder_angle error: %8.3f\n\n', shoulder_angle);
    else
        fprintf('    shoulder_angle error: %8.3f\n\n', shoulder_angle);
        fprintf('    shoulder_angle error: %8.3f\n\n', shoulder_angle);

    
    end
    

    
    % ploting joint angles
	figure(1);clf
    set(gcf,'renderer','painters');
    
    if model.tracking == 4        
        subplot(5,1,1);
        plot(T, X(iy2)*180/pi,'r', 'LineWidth',2);
        ylabel({'ankle'},'fontweight','bold','fontsize',10)
        legend('simulation','data');
        title(' joint angles (rad)')
        
        subplot(5,1,2);
        plot(T, X(iy3)*180/pi,'r', 'LineWidth',2);
        ylabel({'knee'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,3);
        plot(T, X(iy4)*180/pi,'r', 'LineWidth',2);
        ylabel({'hip'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,4);
        plot(T, X(iy5)*180/pi,'r', 'LineWidth',2);
        ylabel({'shoulder'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,5);
        plot(T, X(iy6)*180/pi,'r', 'LineWidth',2);
        hold on; plot(T, ymeas(1,:)*180/pi,'b:' ,'LineWidth',2);
        ylabel({'elbow'},'fontweight','bold','fontsize',10)
        xlabel('time (s)','fontweight','bold','fontsize',10);
        
    elseif model.tracking == 5        
        subplot(5,1,1);
        plot(T, X(iy2)*180/pi,'r', 'LineWidth',2);
        ylabel({'ankle'},'fontweight','bold','fontsize',10)
        legend('simulation','data');
        title(' joint angles (rad)')
        
        subplot(5,1,2);
        plot(T, X(iy3)*180/pi,'r', 'LineWidth',2);
        ylabel({'knee'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,3);
        plot(T, X(iy4)*180/pi,'r', 'LineWidth',2);
        ylabel({'hip'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,4);
        plot(T, X(iy5)*180/pi,'r', 'LineWidth',2);
        hold on; plot(T, ymeas(1,:)*180/pi,'b:' ,'LineWidth',2);
        ylabel({'shoulder'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,5);
        plot(T, X(iy6)*180/pi,'r', 'LineWidth',2);
        hold on; plot(T, ymeas(2,:)*180/pi,'b:' ,'LineWidth',2);
        ylabel({'elbow'},'fontweight','bold','fontsize',10)
        xlabel('time (s)','fontweight','bold','fontsize',10);
    else
        subplot(5,1,1);
        plot(T, X(iy2)*180/pi,'r', 'LineWidth',2);
        hold on; plot(T, ymeas(1,:)*180/pi,'b:' ,'LineWidth',2);
        ylabel({'ankle'},'fontweight','bold','fontsize',10)
        legend('simulation','data');
        title(' joint angles (rad)')
        
        subplot(5,1,2);
        plot(T, X(iy3)*180/pi,'r', 'LineWidth',2);
        hold on; plot(T, ymeas(2,:)*180/pi,'b:' ,'LineWidth',2);
        ylabel({'knee'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,3);
        plot(T, X(iy4)*180/pi,'r', 'LineWidth',2);
        hold on; plot(T, ymeas(3,:)*180/pi,'b:' ,'LineWidth',2);
        ylabel({'hip'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,4);
        plot(T, X(iy5)*180/pi,'r', 'LineWidth',2);
        hold on; plot(T, ymeas(4,:)*180/pi,'b:' ,'LineWidth',2);
        ylabel({'shoulder'},'fontweight','bold','fontsize',10)
        
        subplot(5,1,5);
        plot(T, X(iy6)*180/pi,'r', 'LineWidth',2);
        hold on; plot(T, ymeas(5,:)*180/pi,'b:' ,'LineWidth',2);
        ylabel({'elbow'},'fontweight','bold','fontsize',10)
        xlabel('time (s)','fontweight','bold','fontsize',10);
    end
        
    % ploting machine states
    figure(2);
    set(gcf,'renderer','painters');
    
    
    if model.tracking == 2
        subplot(3,1,1);
        plot(T, X(iy7),'r', 'LineWidth',2);
        ylabel({'flywheel','velocity (m/s)'},'fontweight','bold','fontsize',10)
        legend('simulation');
        
        subplot(3,1,2);
        plot(T, X(iy13),'r', 'LineWidth',2);
        hold on; plot(T, ymeas(6,:),'b:' ,'LineWidth',2);
        ylabel({'force','(kN)'},'fontweight','bold','fontsize',10)
        legend('simulation','data');
    elseif model.tracking == 3
        subplot(3,1,1);
        plot(T, X(iy7),'r', 'LineWidth',2);
        hold on; plot(T, ymeas(6,:),'b:' ,'LineWidth',2);
        ylabel({'flywheel','velocity (m/s)'},'fontweight','bold','fontsize',10)
        legend('simulation','data');
        
        subplot(3,1,2);
        plot(T, X(iy13),'r', 'LineWidth',2);
        hold on; plot(T, ymeas(7,:),'b:' ,'LineWidth',2);
        ylabel({'force','(kN)'},'fontweight','bold','fontsize',10)

    else
        subplot(3,1,1);
        plot(T, X(iy7),'r', 'LineWidth',2);
        ylabel({'flywheel','velocity (m/s)'},'fontweight','bold','fontsize',10)
        legend('simulation');
        
        subplot(3,1,2);
        plot(T, X(iy13),'r', 'LineWidth',2);
        ylabel({'force','(kN)'},'fontweight','bold','fontsize',10)
        legend('simulation');
    end
    

 	subplot(3,1,3);
	plot(T, X(iy1)','r', 'LineWidth',2);
    ylabel({'flywheel','pos (rad)'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);

    % ploting the controlers (torques)  
    figure(3);
    set(gcf,'renderer','painters');
 	subplot(5,1,1);
    plot(T, X(iu1)'*1000,'b', 'LineWidth',2);
    ylabel({'ankle'},'fontweight','bold','fontsize',10)
    title('torques (Nm)')
    
    subplot(5,1,2);
    plot(T, X(iu2)'*1000,'b', 'LineWidth',2);
    ylabel({'knee'},'fontweight','bold','fontsize',10)
    
    subplot(5,1,3);
    plot(T, X(iu3)'*1000,'b', 'LineWidth',2);
    ylabel({'hip'},'fontweight','bold','fontsize',10)
    
    subplot(5,1,4);
    plot(T, X(iu4)'*1000,'b', 'LineWidth',2);
    ylabel({'shoulder'},'fontweight','bold','fontsize',10)
    
    subplot(5,1,5);
    plot(T, X(iu5)'*1000,'b', 'LineWidth',2);
    ylabel({'elbow'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);
    
    anklestd = std(X(iu1)'*1000)
    kneestd = std(X(iu2)'*1000)
    hipstd = std(X(iu3)'*1000)
    shoulderstd = std(X(iu4)'*1000)
    elbowstd = std(X(iu5)'*1000)
    
    

    % ploting cable and rachet machanism 
    figure(4);
    set(gcf,'renderer','painters');
    plot(T,L,'b', 'LineWidth',2)
    ylabel({'L (m)'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);

    
    figure(5);
    set(gcf,'renderer','painters');
    plot(T,L-X(iy1),'b', 'LineWidth',2)
    title('Cable length')
    ylabel({'cable length (m)'},'fontweight','bold','fontsize',10)
    xlabel('time (s)','fontweight','bold','fontsize',10);
    
    
    q = [X(iy2),X(iy3),X(iy4),X(iy5),X(iy6)];						    % extract joint angle trajectories (states 1-5)
    figure(6);
    set(gcf,'renderer','painters');
    animate(q,model.parameters);  	 
    
    
    figure(7)
    plot(T, q*180/pi, 'LineWidth',2);
    legend('q1','q2','q3','q4','q5');
    xlabel('time (s)','fontweight','bold','fontsize',10);
    ylabel('angle (deg)','fontweight','bold','fontsize',10);
    title('Joint Angles','fontweight','bold','fontsize',10)

end