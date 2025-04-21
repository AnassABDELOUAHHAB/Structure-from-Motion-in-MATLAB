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


### 🎯 Bundle Adjustment Cost Function

The optimization problem solved during bundle adjustment minimizes the reprojection error across all visible points in all images. The cost function is defined as:

```math
\min_{\{R_{wj}, t_{wj}\}, \{u_i^w\}} \sum_{j=1}^{M} \sum_{c=1}^{C_j} \left\| p_{j,2D}^{(c)} - K \cdot \pi\left(R_{wj}^\top (u_{3D}^{(c)} - t_{wj})\right) \right\|^2
---


## Visual Results

### Initial Triangulation
![Initial Triangulation](images/triangulation.png)

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
