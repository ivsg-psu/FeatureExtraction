# Feature Extraction

<img src="https://github.com/ivsg-psu/ivsg_master/blob/master/images/FeatureExtraction_cropped.jpg" alt="Feature Extraction" width="1280" height="377">

***
Welcome to the group's repo hub for codes that are used for feature extraction. Specifically, this page links out to repos that includes algorithms built to process large data sets and extract features from these data sets, moving the data from "raw" form into feature-labeled forms.

Note that this repo does NOT include codes that processes or collect raw data; these codes are within the "Field Data Collection" repo. Rather, this area of the team's code is meant to define the procedures and examples that convert data, assuming the data is already collected. Thus, these codes are usually called after those from Field Data Collection are finished.

<!-- TABLE OF CONTENTS -->
## Table of Contents

<details open>
  <summary> Click to see/unsee </summary>

  <ol>
    <li>
      <a href="#data-transforms">Data Transforms</a>
      These are codes that convert data between coordinates, calculate relative velocities, and other mathematical operations on data that are commonly used in the codes below.
    </li>
    <li>
      <a href="#raw-data-processing">Raw Data Processing</a>
      These are codes that convert collected data into "clean" forms, including outlier removal, smoothing data (Kalman filtering), time-correcting data, etc.
    </li>
    <li>
      <a href="#noise-features">Noise Features</a>
      This includes codes that extract statistical noise features from data, including the advanced Allan Variance methods and extrema detection.
    </li>
    <li>
      <a href="#path-features">Path Features</a>
      Paths refer to locations of repeated motion, and these codes show how to extract features of paths include branching points, breaking data into laps, variance changes, offset paths (vehicles changing from one lane to another), abstracting centerline geometries from raw points, elevation, elevation changes, etc. Note that cross-slope is considered a terrain feature (below) and not a path feature since paths are only collections of center-line data.
    </li>
    <li>
      <a href="#vertical-features">Vertical features</a>
      These include vertical profiles of the road such as curbs, potholes, etc.
    </li>
    <li>
      <a href="#lane-boundary-features">Lane Boundary Features</a>
      These include locations of lane markers, patterns of markers, curbs, road edges, etc. that define where lanes start/end laterally
    </li>
    <li>
      <a href="#segment-boundary-features">Segment Boundary Features</a>
      These include locations that define where segments start and end, which are usually where lane marker patterns (solid double yellow) changes to another pattern (striped yellow), where roads intersect or end, (two roads, a driveway, a stop sign), etc.
    </li>
    <li>
      <a href="#visual-features">Visual Features</a>
      These include key road features from images such as pavement color, texture, line colors, signage, etc.
    </li>
    <li>
      <a href="#lidar-features">LIDAR Features</a>
      These extracting key road features from LIDAR sensors such as lines, object profiles and/or boundaries, key points, etc.
    </li>
    <li>
      <a href="#radar-features">RADAR Features</a>
      These extracting key road features from RADAR sensors such as vehicles on the road, velocities, etc.
    </li>
    <li>
      <a href="#terrain-features">Terrain Features</a>
      These features include information about the terrain around the road or on the road surface itself (cross-slope, mesh, etc.), the surrounding area (bridges, guardrails, cross-slopes and side-slopes, etc.) or terrain cues (horizon lines, for example).
    </li>
    <li>
      <a href="#safety-metrics">Safety Metrics</a>
      These include codes to calculate common safety metrics: time-to-collision, nearest collision hazard, etc.
    </li>
    <li>
      <a href="#feature-association-and-abstraction">Feature Association And Abstraction</a>
      These are codes that associate detected features to each other, for example that points detected on one feature match to points on another feature.
    </li>
  </ol>
</details>

<a href="#field-data-collection">Back to top</a>

***

## Data Transforms

<!-- DATA TRANSFORMS -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_DataTransforms_TransformClassLibrary">
      FeatureExtraction_DataTransforms_TransformClassLibrary
      </a>
      <br>
      This is the main Transform class library that contains transformation operations typically needed for cartesian data processing. This includes cartesian translation/rotations from one coordinate system to another (for non-cartesian conversions, example GPS LLA, see the GPS library), calculating relative velocities (wheel velocities from chassis velocities and chassis spin, for example), and converting from/to global to/from sensor coordinates.
    </li>  
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_DataTransforms_GPSMultiAntennaPoseCalibration">
      FeatureExtraction_DataTransforms_GPSMultiAntennaPoseCalibration
      </a>
      <br>
      This is the repo that contains the details of the GPS antenna installation calibration.
    </li>
  </ul>
</details>

<a href="#feature-extraction">Back to top</a>

***

## Raw Data Processing

<!-- RAW DATA PROCESSING -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_DataCleanClassLibrary">
      FeatureExtraction_DataCleanClassLibrary
      </a>
      <br>
      This is the main DataClean class library that contains all the general-use functions to process raw data from the vehicles. It includes outlier removal, time alignment, and Kalman filtering.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_DataClean_BreakDataIntoLaps">
      FeatureExtraction_DataClean_BreakDataIntoLaps
      </a>
      <br>
      This is a repo to demonstrate how to break code up into laps, or segments that are repeated during a data test.
    </li>
    <li>
      <a href="https://www.mathworks.com/content/dam/mathworks/tag-team/Objects/p/preprocessing-time-series-data-tips-and-tricks.pdf">
      Using Timetables in MATLAB to process time data
      </a>
      <br>
      This is a cheat sheet that shows how to merge data from different sensors using MATLAB's timetable functionality, which is useful to perform time alignment.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_ProcessingGPSData_Wahba_Loop_Lane_Centerline_Extraction">
         FeatureExtraction_ProcessingGPSData_Wahba_Loop_Lane_Centerline_Extraction
      </a>
      <br>
      This is an example set of scripts and functions that process raw data from the Wahba loop, primarily GPS position data and some IMU data, by doing the following: query the raw data server to grab raw data, clean up this data using filtering / time-alignment / Kalman Filters, and then averaging paths together. This is a good starter code to practice processing data from the mapping van. It includes 4 laps of the Wahba loop.
      <br>
<img src="https://github.com/ivsg-psu/FeatureExtraction_ProcessingGPSData_Wahba_Loop_Lane_Centerline_Extraction/blob/master/WahbaLoop_v1.png" alt="WahbaLoopV1_Fig" style="width:56px;height:42px;">
    </li>
  </ul>
</details>

<a href="#feature-extraction">Back to top</a>

***

## Noise Features

<!-- NOISE FEATURES -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_AllanVariance_AllanVarianceClassLibrary">
      FeatureExtraction_AllanVariance_AllanVarianceClassLibrary
      </a>
      <br>
      This is the core library to perform Allan Variance noise feature extraction.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_AllanVariance_FAVAR">
      FeatureExtraction_AllanVariance_FAVAR
      </a>
      <br>
      This is the code for FAVAR algorithm.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_AllanVariance_BplusTreeDatabaseImplementation">
      FeatureExtraction_AllanVariance_B<sup>+</sup>treeDatabaseImplementation  
      </a>
      <br>
      This is the code for B<sup>+</sup>-tree implementation of FAVAR.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_AllanVariance_MAfilter">
      FeatureExtraction_AllanVariance_MAfilter
      </a>
      <br>
      This is the code for AVAR analysis of MA filter.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_AllanVariance_FIRandIIRfilters">
      FeatureExtraction_AllanVariance_FIRandIIRfilters
      </a>
      <br>
      This is the code for AVAR analysis of FIR and IIR filters.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_AllanVariance_AnalysisOfPID">
      FeatureExtraction_AllanVariance_AnalysisOfPID
      </a>
      <br>
      This is the code for AVAR analysis of PID controller.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/https-github.com-ivsg-psu-FeatureExtraction_AllanVariance_AnalysisOfILC">
      FeatureExtraction_AllanVariance_AnalysisOfILC
      </a>
      <br>
      This is the code for AVAR analysis of ILC controller.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_NoiseFeatures_Extrema">
      FeatureExtraction_NoiseFeatures_Extrema
      </a>
      <br>
      This code is used to detect extrema in both noisy and noise-free data.
    </li>
  </ul>
</details>

<a href="#feature-extraction">Back to top</a>

***

## Path Features

<!-- PATH FEATURES -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_DataClean_BreakDataIntoLaps">
      FeatureExtraction_DataClean_BreakDataIntoLaps
      >
      </a>
      <br>
      This details the code to break field data into laps. (TO ADD: need more examples to test the function)
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_PathFeatures_ArcFitLibrary">
      FeatureExtraction_PathFeatures_ArcFitLibrary
      >
      </a>
      <br>
      This is the repo that generate best-fit geometries from raw points using connected arcs, spirals, and line segments. In other words, it finds the geometric constructs (arcs, segments, spirals) that fit the curvature in data using arc-fitting methods.
    </li>
  </ul>
</details>

<a href="#feature-extraction">Back to top</a>

***

## Vertical features

***

## Lane Boundary Features

<!-- LANE BOUNDARY FEATURES -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LaneBoundary_extractCL">
      FeatureExtraction_LaneBoundary_extractCL
      </a>
      <br>
      Given the ENU+I point cloud from a LIDAR and the trajectory of the mapping vehicle, finds the centerline (CL) ENU points that best fit a template pattern.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LaneBoundary_FindEdge">
      FeatureExtraction_LaneBoundary_FindEdge
      </a>
      <br>
      Given the XYZ points from a LIDAR and the trajectory of the mapping vehicle, finds the road edge locations on a user-defined search grid.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LaneBoundary_LIDAR_Intensity_and_Height_BodyCentrictoENU">
      FeatureExtraction_LaneBoundary_LIDAR_Intensity_and_Height_BodyCentrictoENU
      </a>
      <br>
      Transforms intensity and height features from body-centric coordinates into GPS ENU coordinate space and plots them.

Designed to work with features from LIDAR data that has been processed with the FeatureExtraction_LaneDetection_LIDAR_Reflectivity_And_Geometry_Lane_Extraction repo from the Feature Association & Abstraction repo hub.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LaneBoundary_Average_Lane_Features">
      FeatureExtraction_LaneBoundary_Average_Lane_Features
      </a>
      <br>
      Finds the average centerline and path of a vehicle over multiple traversals of a road using orthogonal projection*. Examples for averaging functions in the PathPlanning_PathTools_PathClassLibrary repo.

*Designed to work with features from LIDAR data that has been processed with the FeatureExtraction_LaneDetection_LIDAR_Reflectivity_And_Geometry_Lane_Extraction repo from the Feature Association & Abstraction repo hub.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LaneBoundary_ExtremaRoadEdge">
      FeatureExtraction_LaneBoundary_ExtremaRoadEdge
      </a>
      <br>
      Given the XYZ points from a LiDAR and the trajectory of the mapping vehicle, this code finds the road edge locations using an extrema filter.
    </li>
  </ul>
</details>

<a href="#feature-extraction">Back to top</a>
***

## Segment Boundary Features

***

## Visual Features

<!-- VISUAL FEATURES -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <br>
      Pose detection of other vehicles - See the PoseCNN, Mask RCNN, and CozyPose repos on GitHub. Also see work by Luca Carlo (sp?)
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LaneDetection_Image_HSV_Colorspace_Clustering">
      FeatureExtraction_LaneDetection_Image_HSV_Colorspace_Clustering
      </a>
      <br>
      This is the repo that details the efforts in 2021 by Xinyu Cao to detect lane markers in images using clustering of pixel data in the HSV colorspace.
    </li>
    <li>
      <a href="https://github.com/JalaliLabUCLA/Image-feature-detection-using-Phase-Stretch-Transform/tree/master/Matlab">
      Image-feature-detection-using-Phase-Stretch-Transform
      </a>
      <br>
      This is an external site that uses the phase-stretch transform for feature extraction. See the [wikipedia page for Phase stretch transform](https://en.wikipedia.org/wiki/Phase_stretch_transform) for more detail.
    </li>

  </ul>

</details>

<a href="#feature-extraction">Back to top</a>

***

## LIDAR Features

<!-- LIDAR FEATURES -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LaneDetection_LIDAR_Lane_Extraction">
      FeatureExtraction_LaneDetection_LIDAR_Lane_Extraction
      </a>
      <br>
      This repo is for the start of Vahan Kazandjian's honors thesis work that organizes Bobby's basic lane detection and lane following algorithms from LIDAR data and camera images. This is an image-based representation of LIDAR data to perform the alignment of lane markers into paths.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LIDAR_LIDARPoseEstimation">
      FeatureExtraction_LIDAR_LIDARPoseEstimation
      </a>
      <br>
      This is the repo that details the process of estimate the installation position of the Velodyne Lidar.
    </li>
  </ul>

</details>

<a href="#feature-extraction">Back to top</a>

***

## RADAR Features  

***

## Terrain Features

***

## Safety Metrics

<!-- SAFETY METRICS -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_SafetyMetrics_CollisionDetectionFromCircularGeometry">
      CollisionDetectionFromCircularGeometry
      </a>
      <br>
      Methods to calculate collisions using circular path geometry approximations of the vehicle trajectory.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_SafetyMetrics_SafetyMetricsClass">
      FeatureExtraction_SafetyMetrics_SafetyMetricsClass
      </a>
      <br>
      A class library for safety metric calculations including nearest approach, time to collision, etc.
    </li>
  </ul>

</details>

<a href="#feature-extraction">Back to top</a>

***

## Feature Association And Abstraction

<!-- FEATURE ASSOCIATION AND ABSTRACTION -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_CodesRelatingToGeometry_SlopeInterceptFromNPoints">
      FeatureExtraction_CodesRelatingToGeometry_SlopeInterceptFromNPoints
      </a>
      <br>
      This function seeks to fit a line to user-defined points, a well-documented example that is a good start point.
<img src="https://github.com/ivsg-psu/FeatureExtraction_CodesRelatingToGeometry_SlopeInterceptFromNPoints/blob/master/Test2_Fig.png" alt="Test2_Fig" style="width:56px;height:42px;">
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation">
      FeatureExtraction_Association_PointToPointAssociation
      </a>
      <br>
      Codes to perform point-to-point association tests. For example, these can find when a point exists in one XY data set but not the other.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_LaneDetection_LIDAR_Reflectivity_And_Geometry_Lane_Extraction">
      FeatureExtraction_LaneDetection_LIDAR_Reflectivity_And_Geometry_Lane_Extraction
      </a>
      <br>
      This repo is for the later work of Vahan Kazandjian's honors thesis work that uses LIDAR data to extract both image intensity and geometry, and uses both to extract lane center geometry.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_LIDAR_Feature_Template_Map">
      FeatureExtraction_Association_LIDAR_Feature_Template_Map
      </a>
      <br>
Create templates of lateral height and intensity feature locations (relative to the lane centerline*) by saving the features in each scan to templates corresponding to the point on the average centerline path that each scan is closest to**.

*FeatureExtraction_LaneBoundary_Average_Lane_Features is used to define the average lane centerline path.

**Designed to work with features from LIDAR data that has been processed with the FeatureExtraction_LaneDetection_LIDAR_Reflectivity_And_Geometry_Lane_Extraction repo from the Feature Association & Abstraction repo hub.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_LIDAR_Feature_Clustering">
      FeatureExtraction_Association_LIDAR_Feature_Clustering
      </a>
      <br>
      Identifies features that are present in multiple traversals of a road by applying DBSCAN clustering to the features in a map of LIDAR height and intensity templates*. Also, this code applies statistical analysis to the resulting clusters**.

*FeatureExtraction_Association_LIDAR_Feature_Template_Map is used to create the feature template map from LIDAR feature data.

**Designed to work with features from LIDAR data that has been processed with the FeatureExtraction_LaneDetection_LIDAR_Reflectivity_And_Geometry_Lane_Extraction repo from the Feature Association & Abstraction repo hub.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/FeatureExtraction_DataClustering_3DPoints_to_Lines_Arcs_Clothoids">
      FeatureExtraction_DataClustering_3DPoints_to_Lines_Arcs_Clothoids
      </a>
      <br>
      This repo was written by Dan Fescenmyer to process 3D data points into lines, arcs, and clothoids.
    </li>
  </ul>

</details>

<a href="#feature-extraction">Back to top</a>

***

Note: for PhD students, an advanced treatment of sensitivity, observability, and controllability can be found in the article: ["What Is the Adjoint of a Linear System? [Lecture Notes]" by Kouba and Bernstein](https://ieeexplore.ieee.org/document/9094752)

## Methods of System Identification

Feature extraction is a sub-discipline of system identification, about which there are many textbooks. Good articles to read on this topic are given below:

### Nonlinear System Identification

See:["Nonlinear System Identification: A User-Oriented Road Map" by Schoukens and Ljung in 2020](https://ieeexplore.ieee.org/abstract/document/8897147?casa_token=lodiIrETjmYAAAAA:wCL8Pcyj04GQgHJuTr3GKE6uSFEI9uGiRWuMnK7dFSsu1J1G0nf0DE8Hc1bdBSnySYDiiEBo)

or

["Iterative Model Identification of Nonlinear Systems of Unknown Structure: Systematic Data-Based Modeling Utilizing Design of Experiments" by Schrangl, Tkachenko, and Re in 2020](https://ieeexplore.ieee.org/abstract/document/9094757?casa_token=3teGCiMc76UAAAAA:xf29W9seo4fJdPbFIT3ykVfaCraedwR0Te9acpCIvzVsU_qzk_6HsvhN-tx9V1jYKt0s6-9A)
