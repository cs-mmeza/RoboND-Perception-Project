#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    ##############                     ###############
    ##############  FILTERING & RANSAC ###############
    ##############                     ###############

    #################################################################
    ############### Statistical Outlier Filtering ###################
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(1) # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_std_dev_mul_thresh(0.3) # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    cloud_filtered = outlier_filter.filter()

    #################################################################
    ############### Voxel Grid Downsampling #########################
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01                                  #voxel size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE) #set voxel (leaf) size.
    cloud_filtered = vox.filter()

    #################################################################
    ############## PassThrough Filter ###############################
    passthrough_z = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = 0.32
    axis_max = 0.86
    passthrough_z.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough_z.filter()

    passthrough_x = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthrough_x.set_filter_field_name(filter_axis)
    axis_min = 0.35
    axis_max = 0.79
    passthrough_x.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough_x.filter()

    ################################################################
    ############# RANSAC Plane Segmentation ########################
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.015
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    ################################################################
    ############# Extract inliers and outliers #####################
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    #################                    ###################
    ################# Euclidean Clusting ###################
    #################                    ###################

    white_cloud = XYZRGB_to_XYZ(cloud_objects) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

    
    ec = white_cloud.make_EuclideanClusterExtraction() # Create a cluster extraction object

    ec.set_ClusterTolerance(0.026) # Set tolerances for distance threshold 
    ec.set_MinClusterSize(10) # Minimum cluster size (in points)
    ec.set_MaxClusterSize(1000) # Maximum cluster size (in points)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    ####################################################################################
    ############ Cluster-Mask Point Cloud to visualize each cluster separately #########
    
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                    rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    ###########################################################
    ############ Convert PCL data to ROS messages #############
    ros_cloud_objects =  pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    ###########################################################
    ############ Publish ROS messages #########################
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    #################   Object Recognition  ###################
    #################          &            ###################
    #################       Laveling        ###################

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        ####### Grab the points for the cluster from the extracted outliers
        pcl_cluster = cloud_objects.extract(pts_list)

        ####### Convert the cluster from pcl to ROS
        ros_cluster = pcl_to_ros(pcl_cluster)

        ###############################################################
        ############### Extract histogram features #####################
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        ####### Prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        ####### Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        ###### Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass
    
# function to load parameters and request PickPlace service
def pr2_mover(object_list):


    #### Initialize variables
    test_scene_num = Int32()
    test_scene_num.data = 3
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    
    dict_list = []
    i = 0

    #### Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    rospy.loginfo('Starting pr2_mover with {} objects'.format(len(object_list)))

    #### Loop through the pick list
    for target in object_list_param:

        object_name.data = object_list_param[i]['name']
        match_obj = False
        #### Get the PointCloud for a given object and obtain it's centroid
        for detected in object_list:
            if (object_name.data == detected.label):

                points_arr = ros_to_pcl(detected.cloud).to_array()
                centroids = (np.mean(points_arr, axis=0)[:3])

                match_obj = True

        if (match_obj):
            #### Create 'place_pose' for the object
            pick_pose.position.x = np.asscalar(centroids[0])
            pick_pose.position.y = np.asscalar(centroids[1])
            pick_pose.position.z = np.asscalar(centroids[2])

            #### Assign the arm to be used for pick_place
            place_pose.position.x = 0.0
            place_pose.position.z = 0.8

            if (object_list_param[i]['group'] == "red"):
                arm_name.data = "left"         
                place_pose.position.y = 0.71      
            else:
                arm_name.data = "right"
                place_pose.position.y = -0.71

            #### Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_dict = make_yaml_dict(test_scene_num, object_name, arm_name, pick_pose, place_pose)
            dict_list.append(yaml_dict)

            #### Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                #### Message variables to be sent as a service request
                resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

                print ("Response: ",resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        else:
            rospy.loginfo('Can\'t Find Object: {}'.format(object_list_param[i]['name'] ))

        i += 1


    # TODO: Output your request parameters into output yaml file
    send_to_yaml("output_" + str(test_scene_num.data) + ".yaml",dict_list)


if __name__ == '__main__':

    #### ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    #### Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    #### Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    #### Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    #### Initialize color_list
    get_color_list.color_list = []

    #### Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()