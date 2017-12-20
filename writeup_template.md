## Project: Perception Pick & Place
 
-----------------------------------------
[//]: # (Image and equiation References)
[world3]: ./images/world3.png
[image1]: ./images/dust.png
[image2]: ./images/voxel.png
[image3]: ./images/pass_trought.PNG
[image4]: ./images/ransac.png
[image5]: ./images/Clustering2.png
[image6]: ./images/terminal_svm.png
[image7]: ./images/result.png
[matrix1]:./images/confusion_matrices.png


**This is an Udacity project for the Robotics Software Nanodegree Program. the main purpose of the project is to detect different objects with an RGB-D camera. The project is developed in ROS environment and uses gazebo and rviz to simulate the camera and the robot that is supposed to recognize the objects.**

**For this simulation, the objective is to recognice eight diferent objects comming from differents "worlds" in the simulation. (sticky_notes, book, snacks, biscuits, eraser, soap2, soap, glue)**

![simulation_objects][world3]



Before we start filtering the objects, is necessary to train the SVM file to identify each object. We are setting up to [SVM file](https://github.com/csilver2/RoboND-Perception-Project/blob/master/train_svm.py) with linear kernel and the [caputure_feature](https://github.com/csilver2/RoboND-Perception-Project/blob/master/capture_features.py) script to spaw the objects 100 times.

The [features](https://github.com/csilver2/RoboND-Perception-Project/blob/master/features.py) to recognize each object are histograms of the color with 48 bins.
To train the file we run a launcher file named ```robot_spawn.launch``` which spawn each object in the gazebo environment to be recognized and captured in different positions for our SVM file, then we use this script to create confusion matrices that display the results as follows.



![terminal][image6]
![confusion_matrices][matrix1]

### ROS node.

The simulations in ROS include an RGB-D camera able to capture the objects.  To create a filter that labels and recognizes each object, we need to create a node and subscribe to the data creating this topic to the subscriber ```/pr2/world/points```. the image detected for the camera looks like this.

![dust_original][image1]

### Image segmentation. 

_**In this project to recognize the objects in 3D space is necessary to use different algorithms and methods that allow us to separate each object by data points.**_

#### Statistical outliner filtering
The 3D data displayed for the camera contains a lot of "dust" closer to the objects. This makes the objects more difficult to recognize. Thus we would apply different procedures to filter this points.
First, we apply a _statistical outliner filter_ to the image. this filter removes all the points furder from the object which ins this case is the inliner.


#### Voxel
The RGB-D camera gathers a dense point cloud data that can slow down the time of our process, in this case, is recommended to use a voxel grid filter to downgrade the points quantity used in our segmentation process.

![voxel_example][image2]

#### Pass Throuhg Filtering

This function allows us to cut or filter a section of the space through the x, y, and z planes, and we would use it this time to separate the table and the objects from the rest of the stuff around the robot.

#### RANSAC/Segmentation

This algorithm allows us to separate the inliner and outliner data, being the inliner all the points that follow a specific pattern, and outliner the data that does not follow the same pattern. in this case, it separates the points shaping the table and the ones that shape the objects.

![ransac_result][image4]


#### Euclidean clustering

Clustering is necessary to separate the points that shape each object in our point cloud data set.  In this section, we are using DBSCAN algorithm to organize the point cloud corresponding to each object and separating them by color.

![clustering_result][image5]

(Conclusion-n-Comments)

#### Labeling
Ones we can separate each objects by point clouds, we can label the cloud data with the name of each object.

![label_result][image6]
.

#### About the result and the project
The best value that I found to set up the bin histogram is 48, the result is good but even with the 94% of precision and all the time I spent changing the values for the filter, there are two items that the system does not recognise correctly some times in world3 (glue and sticky_notes).the solution for this may be setting up a higher spawning time for capture_feature script.

### Code Index

- Training the data set

	-[feature.py](https://github.com/csilver2/RoboND-Perception-Project/blob/master/features.py)
		*Create the histograms features in this file*
	-[capture_feature.py](https://github.com/csilver2/RoboND-Perception-Project/blob/master/capture_features.py)
		*Take the features and determine the number of times the program will spawn the objects in gazebo*
	-[training.SVM](https://github.com/csilver2/RoboND-Perception-Project/blob/master/train_svm.py)
		*This file set al the features to train the robot with the data*

- ROS node and rviz conection [object_recognision.py](https://github.com/csilver2/RoboND-Perception-Project/blob/master/pr2_robot/scripts/object_recognition.py) (Line 288-316)
	- node (292)
	- subscriber (295)
	- publisher (297-303)
		*sending the results for each filter to rviz*

- RANSAC/Segmentation (53-104)
	- Statistical outliner filtering (60-65)
	- Voxel (67-72)
	- Pass trhough Filtering (74-90)
	- RANSAC (92-99)

	-Displaying outliner/inliner (101-104) 

- Clustering (107-153)

- Object Recognition & labeling (155-202)

- Cloud centroids $ output.yaml(206-286)
	
	- Cloud centroids (248-258)

	- output_*.yaml (285-286) 
		*[output_1.yaml](), [output_2.yaml](), [output_3.yaml]()*













### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



