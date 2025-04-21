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



- **MATLAB** (R2021a or later)  
- Image dataset with pre-computed keypoint tracks.

## Project Overview & Theoretical Background

This project implements an **incremental 3D reconstruction pipeline** from a set of sequential images extracted from a video. The goal is to reconstruct a sparse 3D point cloud of a scene and estimate the camera poses (rotations and translations) corresponding to each image. This technique is widely used in Structure-from-Motion (SfM) and photogrammetry applications.

The reconstruction process relies on key concepts from **multi-view geometry**. Initially, two images are used to estimate their relative pose by decomposing the **fundamental matrix**, assuming known intrinsic parameters. From this, initial 3D points are reconstructed via **triangulation**. A **bundle adjustment** algorithm is then applied to jointly refine the 3D point positions and camera poses by minimizing the **reprojection error**.

As new images are introduced, the process becomes **incremental**:
1. **Localization** – The pose of the new camera is estimated using already triangulated 3D points.
2. **Triangulation** – New 3D points are computed using the newly added view.
3. **Refinement** – A global bundle adjustment is periodically applied to refine all poses and 3D points, using sparse optimization techniques for memory efficiency.

Finally, the reconstruction is visualized using colored 3D point clouds, enhancing both clarity and realism.

---


### Bundle Adjustment Cost Function

The optimization problem solved during bundle adjustment minimizes the reprojection error across all visible points in all images. The cost function is defined as:

$$\min_{\substack{\lbrace {R_{wj}, t_{wj}} \rbrace_{j=1,M} \\ \lbrace {u_i^{w}} \rbrace_{i=1...N} }} \sum_{j=1}^{M} \sum_{c=1}^{C_j} \left\| p_{j,p2DId(c)} - K\pi(R_{wj}^T(u_{p3DId(c)}^w-t_{wj})) \right\|_2^2$$

Where:

- $M$: Number of images (in the initialization step, $M = 2$).
- $C_j$: Number of 3D points observed in image $j$.
- $R_{wj},\ t_{wj}$: Rotation and translation matrices of image $j$.
- $u_i^w$: Reconstructed 3D points in world coordinates.
- $\texttt{p2DId}(c)$: Index of the 2D point among all detected points in image $j$.
- $\texttt{p3DId}(c)$: Index of the corresponding 3D point.
- $\pi(\cdot)$: Projection function that maps 3D points to 2D image coordinates.
- $K$: Intrinsic camera calibration matrix of the form:

$$
K = 
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y
\end{bmatrix}
$$

where $f_x$, $f_y$ are the focal lengths in the $x$ and $y$ directions, and $c_x$, $c_y$ are the coordinates of the optical center in the image.

In this project, the matrix $K$ is defined as:

$$
K = 
\begin{bmatrix}
535 & 0 & 320 \\
0 & 539 & 247
\end{bmatrix}
$$

- $p_{j,\texttt{p2DId}(c)}$: The 2D point in image $j$ corresponding to the projection of the 3D point $u^w_{\texttt{p3DId}(c)}$.
---
### 🧮 Jacobian Matrix Computation

To implement a Levenberg–Marquardt algorithm, it is necessary to know the expression of the derivative of:

$$
K \cdot \pi\left(R_{wj}^\top(u_i^w - t_{wj})\right)
$$

with respect to the parameters $\{R_{wj},\ t_{wj}\}_{j=1}^{M}$ and $\{u_i^w\}_{i=1}^{N}$.

We use the following notations to define the increments estimated at each iteration of Levenberg–Marquardt:

- $R_{wj}^{(l+1)} = R_{wj}^{(l)} \cdot \exp([\delta R_{wj}]^\wedge)$  
- $t_{wj}^{(l+1)} = t_{wj}^{(l)} + \delta t_{wj}$  
- $u_i^{w,(l+1)} = u_i^{w,(l)} + \delta u_i^w$

**Note:** Since a rotation matrix is not an element of Euclidean space but a Lie group, the increment $\delta R_{wj}$ cannot be additive and must be adapted to the geometry of this Lie group. Thus, $\delta R_{wj}$ is a 3D vector (since a rotation matrix has 3 degrees of freedom) and is converted to a rotation matrix using the matrix exponential function (`expm` in MATLAB).

The linearization of the reprojection error is written (omitting indices $l$ and $l+1$):

$$
p_{j,i} - K \cdot \pi\left((R_{wj} \cdot \exp([\delta R_{wj}]^\wedge))^\top (u_i^w + \delta u_i^w - t_{wj} - \delta t_{wj})\right)
\approx r_{j,i} - J_{j,i} \delta
$$

where we define:

- $r_{j,i} = p_{j,i} - K \cdot \pi(R_{wj}^\top (u_i^w - t_{wj}))$

- $\delta =
\begin{bmatrix}
\delta R_{w1} & \delta t_{w1} & \delta R_{w2} & \delta t_{w2} & \cdots & \delta R_{wM} & \delta t_{wM} & \delta u_1^w & \delta u_2^w & \cdots & \delta u_N^w
\end{bmatrix}^\top$

- $J_{j,i} =
\begin{bmatrix}
\begin{bmatrix}
f_x & 0 \\
0 & f_y
\end{bmatrix}
J_\pi
\begin{bmatrix}
J_{R_{w1}} & J_{t_{w1}} & \cdots & J_{R_{wM}} & J_{t_{wM}} & J_{u_1^w} & J_{u_2^w} & \cdots & J_{u_N^w}
\end{bmatrix}
\end{bmatrix}$

with:

$$
J_\pi =
\left.
\frac{\partial}{\partial \delta u_{ji}} \pi(u_{ji} + \delta u_{ji})
\right|_{\delta u_{ji} = 0}
=
\begin{bmatrix}
\frac{1}{z} & 0 & -\frac{x}{z^2} \\
0 & \frac{1}{z} & -\frac{y}{z^2}
\end{bmatrix}
$$

$$
J_{R_{wk}} =
\left.
\frac{\partial}{\partial \delta R_{wk}} \left((R_{wj} \cdot \exp([\delta R_{wj}]^\wedge))^\top (u_i^w - t_{wj})\right)
\right|_{\delta R_{wk} = 0}
=
\begin{cases}
0 & \text{if } k \ne j \\
\left[G_x^\top u_{ji} \quad G_y^\top u_{ji} \quad G_z^\top u_{ji}\right] & \text{if } k = j
\end{cases}
$$

where:

$$
G_x =
\begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & -1 \\
0 & 1 & 0
\end{bmatrix},\quad
G_y =
\begin{bmatrix}
0 & 0 & 1 \\
0 & 0 & 0 \\
-1 & 0 & 0
\end{bmatrix},\quad
G_z =
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
$$

$$
J_{t_{wk}} =
\left.
\frac{\partial}{\partial \delta t_{wk}} \left(R_{wj}^\top (u_i^w - t_{wj} - \delta t_{wj})\right)
\right|_{\delta t_{wk} = 0}
=
\begin{cases}
0 & \text{if } k \ne j \\
- R_{wj}^\top & \text{if } k = j
\end{cases}
$$

$$
J_{u_k^w} =
\left.
\frac{\partial}{\partial \delta u_k^w} \left(R_{wj}^\top (u_i^w + \delta u_i^w - t_{wj})\right)
\right|_{\delta u_k^w = 0}
=
\begin{cases}
0 & \text{if } k \ne i \\
R_{wj}^\top & \text{if } k = i
\end{cases}
$$

---

### 🧠 MATLAB `sparse` Function

Each iteration of the Levenberg–Marquardt algorithm involves solving a problem of the form:

$$
\min_{\delta} \|r - J \delta\|_2^2 + \lambda \|\delta\|_2^2
$$

which is equivalent to solving the linear system:

$$
(J^\top J + \lambda I)\delta = J^\top r
$$

However, when the number of cameras and 3D points becomes large, the matrix $J$ itself becomes very large. It becomes difficult to store it in memory and the computations involving it become very slow.

In practice, to be able to compute a reconstruction with a large number of cameras, it is necessary to **avoid creating the dense matrix $J$**, and instead **construct it in sparse format**.

The `sparse` function allows taking advantage of the fact that matrix $J$ contains a large number of zeros that do not need to be stored.

⚠️ Do **not** use the `sparse` function by passing it a dense matrix $J$, as this would require creating the dense matrix in memory — which is precisely what we want to avoid.

**Instead, proceed in 3 steps:**

1. Create the vectors `i`, `j`, and `v` which represent the **row indices**, **column indices**, and **values** of the non-zero elements of $J$.
2. Fill the vectors `i`, `j`, and `v`.
3. Use them to create the sparse matrix:

```matlab
J = sparse(i, j, v);



---

## Visual Results

### Initial Triangulation
![Initial Triangulation](images/Fonction_cout1.png)

### Bundle Adjustment – Before and After
**Before:**
![Before Bundle Adjustment](images/before_bundle_adjustment.png)

**After:**
![After Bundle Adjustment](images/after_bundle_adjustment.png)

### Incremental Reconstruction with Refinement
![Reconstruction with Refinement](images/refinement.png)

### Final 3D Reconstruction
![Final 3D Reconstruction](images/final_result.png)

> ℹ️ Make sure to place the images inside a folder named `images/` at the root of your repository.
