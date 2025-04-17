# Structure-from-Motion-in-MATLAB
3D Scene Reconstruction from Multiple Images
Incremental Structure from Motion (SfM) for Point Cloud and Camera Pose Estimation

3D Reconstruction Example
Example of 3D point cloud reconstruction from image correspondences.

📖 Project Overview
This MATLAB-based project focuses on 3D scene reconstruction from multiple images by estimating both the 3D point cloud of the scene and the camera poses (rotation and translation matrices). It assumes pre-computed keypoint correspondences (free of outliers) and implements the final stage of the Structure from Motion (SfM) pipeline:

Input: Correspondences between images (keypoints and matches).

Output:

3D point cloud of the scene.

Camera poses (rotation R and translation t) for each image.

✨ Key Features
Incremental Reconstruction: Builds the 3D structure incrementally using successive image pairs.

Reprojection Error Analysis: Quantifies accuracy before and after bundle adjustment.

Flexible Configuration: Adjust the number of images or visualization settings via parameters.

Visualization Options: Choose between MATLAB’s pointCloud() for interactive exploration or plot3() for basic 3D plotting.

🚀 Getting Started
Prerequisites
MATLAB (tested on R2021a or later).

Image dataset with pre-computed keypoint correspondences.

Installation
Clone this repository:

bash
Copy
git clone https://github.com/your-username/TS327_ABDELOUAHHAB_Anass_HADDOUDA_Youssef.git  
Open MATLAB and navigate to the project directory.

🛠 Usage
Running the Reconstruction
Launch the main script:

matlab
Copy
reconstruction_incrementale  
By default, this reconstructs the scene using 30 images.

Outputs:

3D point cloud displayed interactively.

Reprojection errors logged in the MATLAB console.

⚙️ Customization
Key Parameters
Parameter	Location	Description
N_imgs = 30	Line 131	Number of images to use for reconstruction.
m = 10	Line 164	Step size for reprojection error reporting.
Visualization Options
Modify show3D.m to switch between visualization methods:

matlab
Copy
% Use pointCloud() for interactive visualization (lines 21-23):  
ptCloud = pointCloud(Points', 'Color', repmat([255 0 0], size(Points,2), 1));  
viewer = pcplayer([-10 10], [-10 10], [-10 10]);  
view(viewer, ptCloud);  

% OR use plot3() for basic plotting (line 14):  
plot3(Points(1,:), Points(2,:), Points(3,:), 'r.');  
📊 Results
Reprojection Error: Evaluates the accuracy of the estimated camera parameters.

Example output:

Copy
Image 10: Reprojection error before refinement = 1.25 pixels  
Image 10: Reprojection error after refinement = 0.76 pixels  
3D Point Cloud: Visualize the reconstructed scene structure.