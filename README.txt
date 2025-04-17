# README
Ce fichier README a été créé pour vous guider dans l'utilisation du code:


- Vous pouvez lancer directement la reconstruction de 30 images en lançant le code reconstruction_incrementale sans rien changer.

- Nous avons commenté les premiers affichages concernant les deux premières images.
- La ligne 131 permet de spécifier le nombre d'images 3D à reconstruire : N_imgs=30;%size(im_names,1);
- La ligne 164 permet de spécifier le pas pour lequel on affiche l'erreur de reprojection avant et après le raffinement : m=10; (10 par défaut)
- Dans le code show3D vous pouvez choisir la fonction pointCloud()(ligne 21 et 23) ou la fonction plot3() (ligne 14) pour la visualisation.





