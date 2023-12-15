    % Normalized potential field gradient vectors
    function draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits)
        xmin=limits(1);
        xmax=limits(2);
        ymin=limits(3);
        ymax=limits(4);
        X=xmin:0.2:xmax;
        Y=ymin:0.2:ymax;
        [P1,P2]=meshgrid(X,Y);
        
        dobsx=P1-p_obs(1);
        dobsy=P2-p_obs(2);
        % compute gradient vectors at each point of the grid
        VX=-2*alpha*(P1-p_goal(1))+beta*(dobsx)./((dobsx.^2+dobsy.^2).^(3/2));
        VY=-2*alpha*(P2-p_goal(2))+beta*(dobsy)./((dobsx.^2+dobsy.^2).^(3/2));

        % adding repulsive force from p_obs2 if p_obs2 is not equal to p_obs
        if sum(p_obs2==p_obs)~=2
            dobsx2=P1-p_obs2(1);
            dobsy2=P2-p_obs2(2);
            VX=VX+beta*(dobsx2)./((dobsx2.^2+dobsy2.^2).^(3/2));
            VY=VY+beta*(dobsy2)./((dobsx2.^2+dobsy2.^2).^(3/2));
        end

        % normalize the gradient vectors
        VX=VX./sqrt(VX.^2+VY.^2);
        VY=VY./sqrt(VX.^2+VY.^2);
        
        quiver(X,Y,VX,VY);
    end

