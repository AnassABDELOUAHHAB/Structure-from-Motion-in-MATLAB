function [Rwi_c, Twi_c, Uw] = bundleAdjustment_LM_L2_two_views_full(Rwi_c, Twi_c, Uw, tracks_cur, K, p, nTracks, maxIt)
    if(length(Rwi_c)~=2 || length(Twi_c)~=2 || length(tracks_cur)~=2)
        error('only two-view-BA is implemented');
    end   
    lambda = 1e-3;
    lambdaMin = 1e-5;
    lambdaMax = 1e5;
    %compute cost
    c = compute_cost(Rwi_c, Twi_c, Uw, tracks_cur, K, p);
    fprintf('Iter:\t Error:\t \t lambda:\n');
    fprintf('%d\t%f pix \t%f\n',0,c,lambda); 
    for i = 1:maxIt     
        %compute Jacobian and residuals
        r = compute_residuals(Rwi_c, Twi_c, Uw, tracks_cur, K, p);
        J = compute_Jacobian(Rwi_c, Twi_c, Uw, tracks_cur, K, nTracks);
        %update parameters
        [Rwi_c, Twi_c, Uw, c, lambda, success] = updateParameters(Rwi_c, Twi_c, Uw, tracks_cur, K, p, nTracks, r, J, lambda, lambdaMax, lambdaMin, c, i);
        if(~success)
            break;
        end  
    end
end

function c = compute_cost(Rwi_c, Twi_c, Uw, tracks_cur, K, p)
    r = compute_residuals(Rwi_c, Twi_c, Uw, tracks_cur, K, p);
    c = r'*r;
    nb_Pt_2D = 0;
    for i = 1:2 %for each camera    
        nb_Pt_in_current_cam = length(tracks_cur{i}.point3D_ids);
        nb_Pt_2D = nb_Pt_2D + nb_Pt_in_current_cam;
    end
    c = sqrt(c/nb_Pt_2D); %RMSE in pixels
end


function r = compute_residuals(Rwi_c, Twi_c, Uw, tracks_cur, K, p)
    nCam = 2;
    r = [];
    for i = 1:nCam
        p_vis = p{i}(:,tracks_cur{i}.point2D_ids);
        Ui = Rwi_c{i}'*(Uw(:,tracks_cur{i}.point3D_ids) - Twi_c{i});
        p_vis_pred_hom = K*Ui./repmat(Ui(3,:),3,1);
        p_vis_pred = p_vis_pred_hom(1:2,:);
        r = [r; p_vis(:) - p_vis_pred(:)];
    end
end


function J = compute_Jacobian(Rwi_c, Twi_c, Uw, tracks_cur, K, nTracks)
    nCam = 2;
    nb_Pt_2D = 0;
    for i = 1:2 %for each camera    
        nb_Pt_in_current_cam = length(tracks_cur{i}.point3D_ids);
        nb_Pt_2D = nb_Pt_2D + nb_Pt_in_current_cam;
    end
    %J = zeros(2*nb_Pt_2D, 6*nCam+3*nTracks);   
    G1 = [0  0 0; 0 0 -1;  0 1 0];
    G2 = [0  0 1; 0 0  0; -1 0 0];
    G3 = [0 -1 0; 1 0  0;  0 0 0];
    u = [];
    v = [];
    w = [];
    l=0;
    for i = 1:2 %for each camera
        Ui = Rwi_c{i}'*(Uw(:,tracks_cur{i}.point3D_ids) - Twi_c{i});
        nb_Pt_in_current_cam = length(tracks_cur{i}.point3D_ids);
        for t = 1 :nb_Pt_in_current_cam %for each 2D point
            A = [1/Ui(3,t) 0 -Ui(1,t)/(Ui(3,t)^2); 0 1/Ui(3,t) -Ui(2,t)/(Ui(3,t)^2); 0 0 0];
            %3D point derivative
            %J(l:l+1, 1+ 3*(tracks_cur{i}.point3D_ids(t)-1):3*(tracks_cur{i}.point3D_ids(t))) = K(1:2,:)*A*Rwi_c{i}';
            %rotation derivative
            %J(l:l+1,3*nTracks+1+(i-1)*6:3*nTracks+3+(i-1)*6) = [K(1:2,:)*A*G1'*Ui(:,t), K(1:2,:)*A*G2'*Ui(:,t), K(1:2,:)*A*G3'*Ui(:,t)];
            %translation derivative
            %J(l:l+1,3*nTracks+4+(i-1)*6:3*nTracks+i*6) = -K(1:2,:)*A*Rwi_c{i}';
            %l = l+2;
            % 3D point derivative
            Mat = K(1:2,:)*A*Rwi_c{i}';
            values_3D = Mat ;
    
            % Rotation derivative
            values_rot = [K(1:2,:)*A*G1'*Ui(:,t), K(1:2,:)*A*G2'*Ui(:,t), K(1:2,:)*A*G3'*Ui(:,t)];
    
            % Translation derivative
            values_trans = -Mat;

            [x1,y1] = find(values_3D ~= 0);
            [x2,y2] = find(values_rot ~= 0);
            [x3,y3] = find(values_trans ~= 0);
    
            u = [u, (l+x1)', (l+x2)', (l+x3)'];
            v = [v, (y1+3*(tracks_cur{i}.point3D_ids(t) - 1))', (y2+3*nTracks+(i-1)*6)', (y3+3*nTracks+3+(i-1)*6)'];
            w = [w, values_3D(sub2ind(size(values_3D), x1, y1))', values_rot(sub2ind(size(values_rot), x2, y2))', values_trans(sub2ind(size(values_trans), x3, y3))'];
              
            l = l + 2;
        end
    end
    J = sparse(u, v, w, 2*nb_Pt_2D, 6*nCam+3*nTracks);
end

function w_hat = HatSO3(w)
    w_hat = [0 -w(3) w(2);...
             w(3) 0 -w(1);...
            -w(2) w(1) 0];
end
    
function [Rwi_new_c, Twi_new_c, Uw_new, c_new, lambda, success] = updateParameters(Rwi_c, Twi_c, Uw, tracks_cur, K, p, nTracks, r, J, lambda, lambdaMax, lambdaMin, c_prev, iter)
    success = false;
    nCam = 2;  
    while(lambda<lambdaMax)
        %solve linear system
        n=3*nTracks + 6*nCam;
        M = sparse(1:n, 1:n, lambda,n,n);
        delta = mldivide(J'*J + M , J'*r);
        %update variables
        Uw_new = Uw + reshape(delta(1:3*nTracks),3,nTracks);
        Rwi_new_c = cell(1, 2);
        Twi_new_c = cell(1, 2);

        Rwi_new_c{1} = Rwi_c{1}*expm(HatSO3(delta(3*nTracks+1:3*nTracks+3)));
        Twi_new_c{1} = Twi_c{1} + delta(3*nTracks+4:3*nTracks+6);
        
        Rwi_new_c{2} = Rwi_c{2}*expm(HatSO3(delta(3*nTracks+1+6:3*nTracks+3+6)));
        Twi_new_c{2} = Twi_c{2} + delta(3*nTracks+4+6:3*nTracks+6+6);
        
        %compute cost
        c_new = compute_cost(Rwi_new_c, Twi_new_c, Uw_new, tracks_cur, K, p);
        fprintf('%d\t%f pix\t%f\n',iter,c_new,lambda);
        if((c_new+1e-5) < c_prev)
            success = true;
            if(lambda>lambdaMin)
                lambda = lambda/2;
            end
            break;
        else
            lambda = lambda*2;
        end
    end 
    if(~success)
        Rwi_new_c = Rwi_c;
        Twi_new_c = Twi_c;
        Uw_new = Uw;
        c_new = c_prev;
    end
end