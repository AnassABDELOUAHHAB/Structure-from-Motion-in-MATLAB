# Incremental 3D Scene Reconstruction  
**Structure from Motion (SfM) with Bundle Adjustment**  

---

## Project Overview  
This MATLAB-based project implements **incremental 3D reconstruction** from a sequence of images to estimate:  
- **3D point cloud** of the scene  
- **Camera poses** (rotation `R` and translation `t`) for each image  

Built as part of the ENSEIRB Matmeca curriculum, the pipeline follows the Structure from Motion (SfM) methodology, assuming pre-computed keypoint correspondences (outlier-free).  

---

## Key Features  
- **Incremental Reconstruction**: Adds images iteratively while refining poses and 3D points.  
- **Bundle Adjustment**: Minimizes reprojection error using the **Levenberg-Marquardt** algorithm.  
- **Sparse Jacobian Optimization**: Efficiently handles large-scale problems with MATLAB’s `sparse` matrices.  
- **Visualization**: Supports `pointCloud()` for interactive 3D exploration or `plot3()` for basic plotting.  
- **Error Analysis**: Tracks reprojection errors before/after refinement.  

---

## Getting Started  

### Prerequisites  
This sentence uses `$` delimiters to show math inline: $\sqrt{3x-1}+(1+x)^2$
On utilise l'algorithme d'ajustement de faisceaux pour deux images qui est fourni pour ce projet. L'algorithme prend en entrée les points 2D des deux images, les matrices de rotation et de translation des deux images ainsi que les points 3D estimés, et il raffine les valeurs des matrices de rotation et de translation ainsi que les points 3D. L'algorithme d'ajustement de faisceaux consiste à minimiser la fonction de coût suivante\\
\begin{equation}
\min_{\substack{\lbrace {R_{wj}, t_{wj}} \rbrace_{j=1,M} \\ \lbrace {u_i^{w}} \rbrace_{i=1...N} }} \sum_{j=1}^{M} \sum_{c=1}^{C_j} \left\| p_{j,p2DId(c)} - K\pi(R_{wj}^T(u_{p3DId(c)}^w-t_{wj})) \right\|_2^2
\end{equation}\\
Avec : \\
\begin{itemize}
  \item[\textbf{$\bullet$}] M: le nombre d'images à traiter, dans cette étape M=2.
  \item[\textbf{$\bullet$}] $C_j$ le nombre de points 3D vus dans l'image j.
  \item[\textbf{$\bullet$}] $R_{wj}, t_{wj}$ les matrices de rotation et de translation de l'image j.
  \item[\textbf{$\bullet$}] $u_i^{w}$ les points 3D reconstruits.
  \item[\textbf{$\bullet$}] p2DId (c) indique l'indice du point 2D parmi tous les points détectés dans l'image j.
  \item[\textbf{$\bullet$}] p3DId (c) indique l'indice du point 3D.
  \item[\textbf{$\bullet$}] $\pi(.)$ est la fonction de projection.
  \item[\textbf{$\bullet$}] K est la matrice de calibration linéaire de la forme \begin{bmatrix}
  $f_x$ & 0 & $c_x$ \\
  0 & $f_y$ & $c_y$ \\
\end{bmatrix}, avec $f_x$, $f_y$ les composantes en x et y de la distance focale, et $c_x$, $c_y$ les coordonnées du centre optique dans l'image. Dans le projet, on prend : K=\begin{bmatrix}
  535 & 0 & 320 \\
  0 & 539 & 247 \\
\end{bmatrix}
  \item[\textbf{$\bullet$}] $p_{j,p2DId(c)}$ est un point 2D censé
correspondre à la reprojection du point 3D $u_{p3DId(c)}^w$ dans l'image j.

- **MATLAB** (R2021a or later)  
- Image dataset with pre-computed keypoint tracks.