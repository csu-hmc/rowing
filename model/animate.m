function animate(q, par)
% makes an animation of the rower model

% Inputs:
%   q..................(nframes x 5) generalized coordinate trajectories
%   par................(structure) model parameters, see makeparameters.m


    % qd and qdd and xfw don't matter for stick figure
    qd = zeros(size(q));
    qdd = zeros(size(q));
    xfw = 0;
    nframes = size(q,1);
    
    % Prepare the new file
    vidObj = VideoWriter('tmp.avi');
    open(vidObj);
    
    for i = 1:nframes
        % use the rowerdynamics function to get the stick figure
        % coordinates at frame i
        [~,~,~,~,~,~,~,~,s] = rowerdynamics(q(i,:)',qd,qdd,xfw,par);
        clf
        plot(s(:,1),s(:,2),'o-');
        hold on
        
        % draw the seat path
        x = [-1 1];
        y = x*par.a + par.b;
        plot(x,y,'LineWidth',2);
        axis('equal');
        
 % Write each frame to the file
        if i==1
            currFrame = getframe;
        else
            % to make sure the frame is always the same size
            currFrame = getframe(gca,[0 0 vidObj.Width vidObj.Height]);
        end
        writeVideo(vidObj,currFrame);
        pause(0)
    end
% Close the file.
 close(vidObj);
     
    
end