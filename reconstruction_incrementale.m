clear;
close all;
clc;
addpath(genpath('.'));
%% Start the timer
tic;% enregistrement de l'instant actuel pour calculer la durée de la reconstruction 3D
%% Load image names 
load('im_names.mat'); % on charge les noms des images'im_names' pour la reconstruction 3D
                      % afin qu'on puisse les lire en utlisant la fonction "imread()"
%% Load tracks 
load('tracks.mat');% on charge les chemins, les points d'intérêt dans chaque image(p)
                   % et ausssi les matrices R21 et T21 pour
                   % l'initialisation de (Rw1,Tw1) et (Rw2,Tw2)
%% Linear calibration matrix
K = [535 0 320;
    0 539 247;
    0 0 1];%La matrice de calibration

%% Define first pair of images and plot matches 
id_im1 = 1;%on commence par traiter seulement deux images 1 et 2
id_im2 = 2;

im1 = double(imread(fullfile('./images/',im_names{id_im1})))/255;
im2 = double(imread(fullfile('./images/',im_names{id_im2})))/255;

%p1 = p{id_im1};%les points d'intérêt dans l'image 1
%p2 = p{id_im2};%les points d'intérêt dans l'image 2

%[Li1, Loc2] = ismember(tracks{id_im1}.point3D_ids, tracks{id_im2}.point3D_ids);
%matches = [tracks{id_im1}.point2D_ids(Li1)'; tracks{id_im2}.point2D_ids(Loc2(Li1))'];

%figure(2),
%plotmatches(im1,im2,p1,p2,matches,'interactive',0);
%title('Matches between im1 and im2');
%drawnow;

%% Initialization
Rw1 = eye(3);% on initialise les matrices de rotation/translation des deux caméras 1 et 2
Tw1 = zeros(3,1);

Rw2 = Rw1*(R21');
Tw2 = -Rw2*T21 + Tw1;
%{
    En utilisant la fonction ismember() on cherche les correspendances entre 
    les deux première images, ceci en cherchant les ids identiques des pts 3D
    entre les deux images qui nous permettent de sélectionner parmi tous
    les points d'intérêt 2D ceux qui sont en correspendance (p1 et p2).
%}

[Li1, Loc2] = ismember(tracks{id_im1}.point3D_ids, tracks{id_im2}.point3D_ids);
matches = [tracks{id_im1}.point2D_ids(Li1)'; tracks{id_im2}.point2D_ids(Loc2(Li1))'];

inv_K=inv(K);
p1 = p{1}(:,matches(1,:));
p1_hom = [p1; ones(1,size(p1,2))];
m1_hom = inv_K*p1_hom;
p2 = p{2}(:,matches(2,:));
p2_hom = [p2; ones(1,size(p2,2))];
m2_hom = inv_K*p2_hom;

%Triangulation des points 3D vus par les deux caméras 1 et 2 (points rouges et noirs selon le cours)
Uw = triangulate(m1_hom, m2_hom, Rw1, Tw1, Rw2, Tw2);%les premiers poits 3d reconstruits
disp(size(Uw))
Rwi_c = {Rw1, Rw2};
Twi_c = {Tw1, Tw2};
%enregistrement de la couleur de chaque point d'intérêt dans p1 ou p2 (les couleurs sont identiques)
Rwi_BA_colors=zeros(size(p1,2),3);
for pt = 1:size(p1,2)
    Rwi_BA_colors(pt,1) =im2(floor(p1(2,pt)),floor(p1(1,pt)),1);
    Rwi_BA_colors(pt,2) =im2(floor(p1(2,pt)),floor(p1(1,pt)),2);
    Rwi_BA_colors(pt,3) =im2(floor(p1(2,pt)),floor(p1(1,pt)),3);
end
val_init=size(p1,2)+1;
%% Visualize 3D scene
im_names_c = {fullfile('./images/',im_names{id_im1}),fullfile('./images/',im_names{id_im2})};
%figure(3), show3D(Rwi_c, Twi_c, K, Uw,Rwi_BA_colors, im_names_c); title('3D view before BA');
%drawnow;


%% Bundle Adjustment

%define tracks currently used and define new point3D_ids
tracks_cur = {tracks{id_im1}, tracks{id_im2}};
nTracks = sum(Li1);
tracks_cur{1}.oldpoint3D_ids = tracks_cur{1}.point3D_ids(Li1);
tracks_cur{1}.point2D_ids = tracks_cur{1}.point2D_ids(Li1);
tracks_cur{1}.point3D_ids = 1:nTracks;

tracks_cur{2}.oldpoint3D_ids = tracks_cur{2}.point3D_ids(Loc2(Li1));
tracks_cur{2}.point2D_ids = tracks_cur{2}.point2D_ids(Loc2(Li1));
tracks_cur{2}.point3D_ids = 1:nTracks;

%% Visualize reprojection errors before Bundle Adjustment
%{
figure(4)
subplot(121);
p1_vis = p{1}(:,tracks_cur{1}.point2D_ids);
plotReprojectionError(im1,p1_vis,Uw,Rwi_c{1},Twi_c{1},K);
title('Reproj. errors in im1 before BA');

subplot(122);
p2_vis = p{2}(:,tracks_cur{2}.point2D_ids);
plotReprojectionError(im2,p2_vis,Uw,Rwi_c{2},Twi_c{2},K);
title('Reproj. errors in im2 before BA');
drawnow;
%}
%run BA
maxIt = 100;
[Rwi_BA_c, Twi_BA_c, Uw_BA] = bundleAdjustment_LM_L2_two_views_full(Rwi_c, Twi_c, Uw, tracks_cur, K, p, nTracks, maxIt);

%{
%Visualize reprojection errors after Bundle Adjustment
figure(5)
subplot(121);
imshow(im1);
hold on;
p1_vis = p{1}(:,tracks_cur{1}.point2D_ids);
plotReprojectionError(im1,p1_vis,Uw_BA,Rwi_BA_c{1},Twi_BA_c{1},K);
title('Reproj. errors in im1 after BA');

subplot(122);
imshow(im2);
hold on;
p2_vis = p{2}(:,tracks_cur{2}.point2D_ids);
plotReprojectionError(im2,p2_vis,Uw_BA,Rwi_BA_c{2},Twi_BA_c{2},K);
title('Reproj. errors in im2 after BA');
drawnow;
%}

%% reconstruction incémentale: l'ajout des images restantes
N_imgs=30;%size(im_names,1);
for i=3:N_imgs
    disp(['Ajout de l''image ' num2str(i)]);
    im_names_c{i}=fullfile('./images/',im_names{i});
    im_i = double(imread(fullfile('./images/',im_names{i})))/255;
    %{
        En utilisant la fonction ismember() on cherche les correspendances entre 
        l'image actuelle et tous les points 3D déjà reconstruits, .
    %}
    [Li1, Loc2] = ismember(tracks_cur{i-1}.oldpoint3D_ids,tracks{i}.point3D_ids);
    pts_3d_communs = tracks{i}.point3D_ids(Loc2(Li1));
    nTracks=sum(Li1);  
    tracks_cur{i} = tracks{i};  
    tracks_cur{i}.point2D_ids = tracks{i}.point2D_ids(Loc2(Li1));
    tracks_cur{i}.point3D_ids=1:nTracks;
    
    % on extrait les points 3D communs entre les trois images(points rouges selon le cours)
    Uw= Uw_BA(:,tracks_cur{i-1}.point3D_ids(Li1));
    %run simple BA
    %{
       Estimation de Rw3, tw3 en minimisant l'erreur de reprojection de cette 3ème caméra par rapport
       aux points 3D déjà reconstruits (les points 3D sont figés durant cette étape). Il s'agit donc
       ici d'appliquer un algorithme d'ajustement de faisceaux simplifié car uniquement Rw3 et tw3
       sont optimisés. Les paramètres Rw3 et tw3 sont initialisés avec les valeurs de Rw2 et tw2.
    %}
    [Rwi_newCam, Twi_newCam] = bundleAdjustment_LM_L2_two_views_simple(Rwi_BA_c{i-1}, Twi_BA_c{i-1}, Uw, tracks_cur{i}, K, p, nTracks, maxIt);
    Rwi_BA_c{i}=Rwi_newCam;
    Twi_BA_c{i}=Twi_newCam; 

    pts_3d_construit=pts_3d_communs;
    tracks_cur{i}.point3D_ids = tracks_cur{i-1}.point3D_ids(Li1);

    %Triangulation des points 3D qui peuvent l'être grâce à l'ajout de l'image actuelle((chemins ou pts bleus et verts selon le cours).
    m=10;
    n = mod(i,m);
    for j=1:i-1
        
        [Li1, ~] = ismember(tracks{j}.point3D_ids, pts_3d_communs);
        pts_bleus_verts3D = tracks{j}.point3D_ids(~Li1);
        pts_bleus_verts2D = tracks{j}.point2D_ids(~Li1);

        [Li1, Loc2] = ismember(pts_bleus_verts3D, tracks{i}.point3D_ids);
        if sum(Li1) == 0
            continue
        end
        pts_3d_construit = [pts_3d_construit;pts_bleus_verts3D(Li1)];

       
        er=size(Uw_BA,2)+1:(size(Uw_BA,2)+size(pts_bleus_verts3D(Li1),1));

        tracks_cur{i}.point3D_ids = [tracks_cur{i}.point3D_ids,er];
        tracks_cur{i}.point2D_ids = [tracks_cur{i}.point2D_ids;tracks{i}.point2D_ids(Loc2(Li1))];

        tracks_cur{j}.oldpoint3D_ids = [tracks_cur{j}.oldpoint3D_ids;pts_bleus_verts3D(Li1)];
        tracks_cur{j}.point2D_ids = [tracks_cur{j}.point2D_ids;pts_bleus_verts2D(Li1)]; 
        tracks_cur{j}.point3D_ids = [tracks_cur{j}.point3D_ids,er];
        
        matches = [pts_bleus_verts2D(Li1)'; tracks{i}.point2D_ids(Loc2(Li1))']; 

        p1 = p{j}(:,matches(1,:));
        p1_hom = [p1; ones(1,size(p1,2))];
        m1_hom = inv_K *p1_hom;
        p2 = p{i}(:,matches(2,:));
        p2_hom = [p2; ones(1,size(p2,2))];
        m2_hom = inv_K *p2_hom;
        Uw_BA = [Uw_BA,triangulate(m1_hom, m2_hom, Rwi_BA_c{j}, Twi_BA_c{j},Rwi_newCam, Twi_newCam)];

        %enregistrement des couleurs
        Rwi_BA_colors=[Rwi_BA_colors;zeros(size(p2,2),3)];
        pt=1;
        for o = val_init:val_init+size(p2,2)-1
            Rwi_BA_colors(o,1) =im_i(floor(p2(2,pt)),floor(p2(1,pt)),1);
            Rwi_BA_colors(o,2) =im_i(floor(p2(2,pt)),floor(p2(1,pt)),2);
            Rwi_BA_colors(o,3) =im_i(floor(p2(2,pt)),floor(p2(1,pt)),3);
            pt=pt+1;
        end
        val_init=val_init+size(p2,2);

    end
    tracks_cur{i}.oldpoint3D_ids = pts_3d_construit;

    %RAFFINEMENT
    if n == 0
        figure,% avec un pas de 10 images, on affiche les erreurs de reprojection ,avant et apres l etape de raffinement, des points 3d contruits dans l'image i
        subplot(121)
        pi_vis = p{i}(:,tracks_cur{i}.point2D_ids);
        plotReprojectionError(im_i,pi_vis,Uw_BA(:,tracks_cur{i}.point3D_ids),Rwi_BA_c{i},Twi_BA_c{i},K);
        titre = ['Reproj. errors before BA in im', num2str(i)];
        title(titre);

    end

    disp('Raffinement')% on fait un raffinement sur toutes les images ajoutées
    [Rwi_BA_c, Twi_BA_c, Uw_BA] = bundleAdjustment_LM_n_views_full(Rwi_BA_c, Twi_BA_c, Uw_BA, tracks_cur, K, p, size(Uw_BA,2), maxIt);

    if n == 0
        subplot(122);
        pi_vis = p{i}(:,tracks_cur{i}.point2D_ids);
        plotReprojectionError(im_i,pi_vis,Uw_BA(:,tracks_cur{i}.point3D_ids),Rwi_BA_c{i},Twi_BA_c{i},K);
        titre = ['Reproj. errors after BA in im', num2str(i)];
        title(titre); 
    end 
 
end

%% Visualisation
%{
% détermination du barycentre des pts 3d
N=size(Uw_BA,2);
x=sum(Uw_BA(1,:))/N;
y=sum(Uw_BA(2,:))/N;
z=sum(Uw_BA(3,:))/N;

Uw_BA = Uw_BA-[x;y;z];
%}
%% Visualize refined 3D scene
figure, show3D(Rwi_BA_c, Twi_BA_c, K, Uw_BA,Rwi_BA_colors, im_names_c); title('3D view after BA');
drawnow;
%save....


%nous avons ajouté cette partie parce que les points 3d sont regroupé en
%deux regions dense ce qui affecte la barycentre de tout les point. Nous
%avons ajouté une classification Kmeans qui permet de rémidier à ce problème
%numClusters = 2;
%[idx, centroids] = kmeans(Uw_BA', numClusters);
% Calculate the centroid for the more dense region
%denseCentroid = mean(centroids, 1);
%centroid = mean(Uw_BA,2);
%Uw_BA = Uw_BA-denseCentroid';
%figure, show3D(Rwi_BA_c, Twi_BA_c, K, Uw_BA,Rwi_BA_colors, im_names_c); title('3D view after BA');
%drawnow;
%save....

% Stop the timer
elapsedTime = toc;
disp(['Elapsed time: ' num2str(elapsedTime/60 ) ' min']);

