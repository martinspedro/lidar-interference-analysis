Thesis Writing Plan
===================

O objetivo principal da tese é mostrar um estudo sobre interferência entre LiDARs.
Contudo, o trabalho desenvolvido não vai direto a esta caracterização da interferência, com a premissa de perceber como
é que essa interferência afeta objetos de interesse na point cloud. Para isso, é necessário calibrar uma câmara e um
LIDAR, fundir as suas informações, detetar objetos em imagem e obter point cloud individuais correspondente a cada
objeto.

A minha ideia para o flow da tese é começar por explicar que se começa por experimentar fusão sensorial entre câmara e
LIDAR, usando datasets já calibrados. Depois, passa-se a implementar uma calibração que permita fusão sensorial dos
dados experimentais obtidos. O próximo passo consiste na deteção de objetos na câmara e depois na segmentaçãp da point
cloud apenas na zona de interesse para esse objeto.

Isto permite que seja possível, no final, não só caracterizar a point cloud de um ponto de vista estatistico (que é o
que tenho até agora - percentagem de pontos interferidos, quais as linhas e ângulos mais interferidos, variação de
intensidade, etc.). Mas também analisar a interferência de certos objetos de interesse, como cadeiras, caixas, bolas de
futebol e pessoas.

Status:
  - Thesis has three main areas of Work
    - Calibration + Sensor fusion (Implementation Done. Documentation Halfway. Some Camera Ready Results)
    - Object Detection in Images, transposition to point cloud and Point Cloud Segmentation (Implementation Halfway. No Documentation. No Camera Ready Results)
    - Study on LiDAR Interference (Implementation almost done. Documentation Halfway. Some Camera Ready Results)


Thesis Layout
===================

- Cover
- Dissertation title
- Jury
- Acknowledges
- Keywords and Abstract (pt, en)
- Contents
- List of Figures
- List of tables
- List of Acronyms

# 1. Introduction
- Summary of the topic
- What is the problem
- Why it matters
- key sensors para ADAS (cam, LIDAR, etc.)
- Key concepts:
    - Road accidents
    - ADAS
    - Self-driving
		- Small Description of the types of sensors used (Ultrasons, RADAR, Camera, LIDAR)
    - LiDAR
    - LiDAR Interference (?)

## 1.1 Scope and Motivation
- What is the problem 
- How am I going to try to solve it
- Why is the topic important
- Motivation & Relevance for and to the problem
- Why and how it matters

## 1.2 Objectives
- Study LiDAR Interference
- Create a dataset with multiple scenarios and test conditions to study LiDAR interference
- Calibration between LiDAR and Camera
- Augmenting the information used on point cloud mapping, through sensor fusion between LiDAR and Camera,
- Object Detection in Images
- Images and Point Cloud correspondence between objects, with and without LIDAR interference

## 1.3 Document Organization
- How is the thesis structured
    - Headings of this document

## 1.4 Contributions
- Indicate where the work has been presented before the thesis defence
- Students@DETI (poster on sensor fusion)
- RECPAD (if accepted, paper on LiDAR Interference and poster or oral presentation)
- Others (if applicable)

# 3. State of the Art
## 3.1 Datasets
## 3.2 Camera
## 3.3 LiDAR
## 3.4 Camera and LiDAR Extrinsic Calibration
## 3.5 Sensor Fusion
- What is sensor fusion and what are its advantages?
- Requirements for sensor Fusion:
    - Data must be on the same referential
    - Why Udacity was the first to be chosen as a dataset and then was abandoned for Kitti
-
## 3.6 Object Detection
- On Image: YOLO, Fast-CNN, Faster-CNN, etc.
- Correspondence between images and point cloud

## 3.7 LiDAR Interference


# 4. Intrinsic and Extrinsic Calibration
## 4.1 Intrinsic Camera calibration
- Calibration Patterns
- Experimental Setup
- How it was done
- Results

## 4.2 Intrinsic LiDAR Calibration
- How it was done
- Results

## 4.3 Extrinsic Camera and LiDAR Calibration
- Mathematical Principle (referential, 6DOF, Rigid Body Transforms)
- Experimental Setup
- How to calibrate (tutorial like)
- Calibration method implemented
- Results
- Test with sensor fusion (apply sensor fusion to calibrated experimental data)

## 4.4 Final Remarks

# 5. Sensor Fusion
- Mathematical Principle
- How it was implemented
- Results
- Final Remarks


# 6. Object Detection
## 6.1 On Image
- Results

## 6.2 Correspondence between bounding boxes on image and point cloud
- Mathematical Principle
- How it was Done
- Implementation
- Object Segmentation
- Results

## 6.3 Final Remarks

# 7. LiDAR Interference
## 7.1 Experimental Setup
## 7.2 Ground Truth Generation
## 7.3 Voxel-ize analysis
## 7.4 Point-to-point analysis
## 7.5 Object Interference

# 8. Conclusions and Future Work
## 8.1 Conclusions
## 8.2 Future Work

# Bibliography

# Appendices
- Papers from Original Contributions
- LIDAR Intrinsic Calibration data (?)
- Camera Intrinsic Calibration Matrices (?)
