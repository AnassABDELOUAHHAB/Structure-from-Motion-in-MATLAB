function [] = show3D(Rwi_c, Twi_c, K, Uw,Rwi_BA_colors, im_names_c)

    nbIm = length(im_names_c);

    % plot 3D points
    grid on;
    axis equal;
    cameratoolbar;
    cameratoolbar('SetCoordSys','y');
    hold on;
    
    if(~isempty(Uw))
        %for pt = 1:size(Uw, 2)
            %plot3(Uw(1, pt), Uw(2, pt), Uw(3, pt), '.', 'Color', Rwi_BA_colors(pt, :));
            %scatter3(Uw(1, pt), Uw(2, pt), Uw(3, pt), 'Marker', '.', 'CData', Rwi_BA_colors(pt, :));
            %ptCloud = pointCloud(Uw(:, pt)', 'Color', Rwi_BA_colors(pt, :));
            
            %hold on;
        %end
        
        ptCloud = pointCloud(Uw', 'Color', Rwi_BA_colors);
    end
    pcshow(ptCloud);
    
    for i = 1:nbIm
       
        %load image
        I = imread(im_names_c{i});
        [h,w,~] = size(I);
        
        I = imresize(I, 0.2);
        
        Rwi = Rwi_c{i};
        Twi = Twi_c{i};
        
        p_corners_hom = [1 w w 1; 1 1 h h; 1 1 1 1];
        m_corners_hom = inv(K)*p_corners_hom;
        
        m_corners_hom_in_w = Rwi*m_corners_hom + Twi;
        
        %plot camera frustum
        plot3([m_corners_hom_in_w(1,:), m_corners_hom_in_w(1,1)],...
            [m_corners_hom_in_w(2,:), m_corners_hom_in_w(2,1)],...
            [m_corners_hom_in_w(3,:), m_corners_hom_in_w(3,1)],'-');
        
        plot3([m_corners_hom_in_w(1,:), m_corners_hom_in_w(1,1)],...
            [m_corners_hom_in_w(2,:), m_corners_hom_in_w(2,1)],...
            [m_corners_hom_in_w(3,:), m_corners_hom_in_w(3,1)],'-');
        
        %plot camera frame
        plot3([Twi(1), Twi(1)+Rwi(1,1)],...
            [Twi(2), Twi(2)+Rwi(2,1)],...
            [Twi(3), Twi(3)+Rwi(3,1)],'-r');
        
        plot3([Twi(1), Twi(1)+Rwi(1,2)],...
            [Twi(2), Twi(2)+Rwi(2,2)],...
            [Twi(3), Twi(3)+Rwi(3,2)],'-g');
        
        plot3([Twi(1), Twi(1)+Rwi(1,3)],...
            [Twi(2), Twi(2)+Rwi(2,3)],...
            [Twi(3), Twi(3)+Rwi(3,3)],'-b');
        
        %plot images
        surface([m_corners_hom_in_w(1,1) m_corners_hom_in_w(1,2); m_corners_hom_in_w(1,4) m_corners_hom_in_w(1,3)], ...
            [m_corners_hom_in_w(2,1) m_corners_hom_in_w(2,2); m_corners_hom_in_w(2,4) m_corners_hom_in_w(2,3)], ...
            [m_corners_hom_in_w(3,1) m_corners_hom_in_w(3,2); m_corners_hom_in_w(3,4) m_corners_hom_in_w(3,3)], ...
            'FaceColor', 'texturemap', 'CData', I );
       
    end
    
end