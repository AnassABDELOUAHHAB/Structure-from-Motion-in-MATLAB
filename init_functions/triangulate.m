function Uw = triangulate(m1_hom, m2_hom, Rw1, Tw1, Rw2, Tw2)
    nbPt = size(m1_hom,2);
    Uw = zeros(3,nbPt);
    R1w = Rw1';
    T1w = -R1w*Tw1;
    R2w = Rw2';
    T2w = -R2w*Tw2;
    for i = 1:nbPt
        
        m1 = m1_hom(:,i);
        m2 = m2_hom(:,i);
        
        A1 = [m1(1)*R1w(3,:) - R1w(1,:); m1(2)*R1w(3,:) - R1w(2,:)];
        b1 = [T1w(1) - m1(1)*T1w(3); T1w(2) - m1(2)*T1w(3)];
        
        A2 = [m2(1)*R2w(3,:) - R2w(1,:); m2(2)*R2w(3,:) - R2w(2,:)];
        b2 = [T2w(1) - m2(1)*T2w(3); T2w(2) - m2(2)*T2w(3)];
        
        Uw(:,i) = mldivide([A1;A2], [b1;b2]);
        
    end
end