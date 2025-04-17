function plotReprojectionError(im,p_i,Uw,Rwi,Twi,K)
    imshow(im);
    hold on;
    plot(p_i(1,:),p_i(2,:),'or');
    Ui = Rwi'*(Uw - Twi);
    p_i_pred = K*Ui./repmat(Ui(3,:),3,1);
    plot(p_i_pred(1,:),p_i_pred(2,:),'xb');
    line([p_i(1,:);p_i_pred(1,:)],[p_i(2,:);p_i_pred(2,:)]);
end 