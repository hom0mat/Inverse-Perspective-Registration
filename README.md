# ✨ CAD Model Pose Alignment: Inverse Perspective Matching ✨

This repository contains the implementation of an **Iterative Inverse Perspective Matching** pipeline for aligning CAD models to image data.

## 🌟 Overview

The goal of this algorithm is to estimate the 3D pose (rotation and translation) of a CAD model based on a single 2D image. By combining **edge detection** and **iterative geometric alignment**, the system can project the CAD model into the image space and refine its pose until it accurately matches the observed scene.

Our implementation is inspired by:
> P. Wunsch and G. Hirzinger,  
 “Registration of CAD-models to images by iterative inverse perspective matching,” ICPR 1996.

## 🔧 Algorithm Pipeline

1. **Image Preprocessing**  
   - Convert input image to grayscale.  
   - Apply Canny edge detection to extract meaningful edges.

2. **Inverse Ray Generation**  
   - Generate normalized 3D rays from edge coordinates, centered on the camera.

3. **CAD Model Loading**  
   - Load the `.obj` CAD model using `trimesh` or similar libraries.

4. **Matching and Correspondence**  
   - For each inverse ray, find the closest CAD model vertex.  
   - Project CAD points along the ray direction.

5. **Rigid Transformation Computation**  
   - Use Singular Value Decomposition (SVD) to estimate the optimal rotation and translation between model and scene.

6. **ICP Iterative Refinement**  
   - Update correspondences and refine transformation until convergence.

7. **Result Export and Simulation Integration**  
   - Save the final pose transformation as a JSON file.  
   - This JSON can be read by a physics or graphics simulator where the CAD model is also loaded.  
   - The simulator uses the JSON data to animate or move the CAD model toward the pose estimated from the image and edge detection, allowing visual verification of the alignment process.


## 📈 Experimental Highlights

- Reference image: `cad_wrench.png`  
- CAD model: `wrench.obj`  
- Output: JSON file including aligned pose and transformation data (rotation + translation)

## 💬 Academic Notes

This approach merges:
- Classical computer vision tools (Canny edge detection)  
- Geometric optimization (ICP, SVD)  
- Model-based registration (using known CAD geometry)

It is especially useful for robotics, augmented reality, and teleoperation pipelines, where precise spatial alignment is critical.

In this implementation, Processing is applied to read the JSON file. The required code is in the folder reto_2.

