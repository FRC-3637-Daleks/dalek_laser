#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include <laser_geometry/laser_geometry.h>

#include <nav_msgs/OccupancyGrid.h>

#include <std_msgs/Header.h>

#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>

#include <cmath>
#include "Eigen/Dense"
#include <ctime>    

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h> // Include for tf::Transform

#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <tf2_eigen/tf2_eigen.h>

#include <tf2_ros/transform_listener.h>  // For TF buffer and listener
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2::doTransform with geometry_msgs
#include <tf2/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
//#include <transform_point_cloud/LookupTransformConfig.h>

//TF code: https://github.com/lucasw/transform_point_cloud/blob/master/src/transform_point_cloud.cpp

class Node {
private:
    //Not required here, used for conversion to pcl PointCloud2 in scanToFilterPC2()
	laser_geometry::LaserProjection projector_;

    //Stores the transformation compaired to map
    Eigen::Matrix4f cumulativeTransform;

    //Contains the map as a PC
    pcl::PointCloud<pcl::PointXYZ>::Ptr map;

    //PointCloud2 buffer
    pcl::PCLPointCloud2 scanBuffer;
    sensor_msgs::LaserScan::ConstPtr msg_storage;

    //Class for dealing with publishing & subscribing
	ros::NodeHandle nh;

    //Subs
    ros::Subscriber s_scan;
    ros::Subscriber s_map;

    //Pub
    ros::Publisher p_voxelPC2;
    ros::Publisher p_adjustedScan;
    ros::Publisher p_PCMap;

    //TF Shenanogans
    tf2::BufferCore tfBuffer;
    tf::TransformListener tfListener;  // Add TF Listener
    tf::TransformBroadcaster broadcaster;
    //tf2_ros::Buffer tf_buffer;
    

    //Robot Frame and Odom Frame Variables
    std::string targetFrame = "odom"; // Or whatever your odometry frame is named
    std::string sourceFrame = "laser"; // Or whatever your laser frame is named


    //Configs
    float leaf_size = 0.05; //used for voxel filter
    int maxIterations = 10; //Amount of times ICP itterates
    double MaxCorrespondenceDistance = 2.0; //Max range from pevious scan ICP cares about
public:

    Node();
    ~Node();

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg);

    void ICPCalc(pcl::PCLPointCloud2 cloudPeram);

    pcl::PCLPointCloud2 scanToPC2(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*scan_msg, cloud);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(cloud, pcl_pc2);

        return pcl_pc2;
    }

    pcl::PCLPointCloud2 filterPC2(pcl::PCLPointCloud2 pc2) {
        pcl::PCLPointCloud2 cloud_filtered;  // output cloud

        // Create the filtering object
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(pc2)); // Convert const ref to shared_ptr
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        sor.filter(cloud_filtered);

        return cloud_filtered;
    }

    void publishPC2(ros::Publisher pub, pcl::PCLPointCloud2 pc2) {
        //From pcl PointCloud2 to PointCloud 
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pc2, *temp_cloud);

        //pcl PointCloud to ROS PointCloud
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*temp_cloud, output);

        output.header.stamp = ros::Time::now();
        output.header.frame_id = "odom";

        pub.publish(output);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr GridToPC2(const nav_msgs::OccupancyGridConstPtr& map_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Get map information
        double resolution = map_msg->info.resolution;
        int width = map_msg->info.width;
        int height = map_msg->info.height;
        double origin_x = map_msg->info.origin.position.x;
        double origin_y = map_msg->info.origin.position.y;

        // Iterate over each cell in the occupancy grid
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Get the occupancy value at the current cell
                int occupancy_value = map_msg->data[y * width + x];

                // Check if the cell is occupied (typically values > 0 represent obstacles)
                if (occupancy_value > 0) {
                    // Convert cell coordinates to world coordinates
                    double world_x = origin_x + (x * resolution);
                    double world_y = origin_y + (y * resolution);

                    // Create a point and add it to the cloud
                    pcl::PointXYZ point;
                    point.x = world_x;
                    point.y = world_y;
                    point.z = 0.0;  // Assuming a 2D map
                    cloud->push_back(point);
                }
            }
        }
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterStat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

        sor.setInputCloud(cloudIn);
        sor.setMeanK (50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*filtered_cloud);
        
        return filtered_cloud;
    }

    
    //map->laser
    //the tf listner should be connected to buffer
    //try to use only tf listner to turn cloud
    //sensor_msgs::LaserScan::ConstPtr transformCloud(const sensor_msgs::LaserScan::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
        //Map is generated in odom frame
        // auto startTime = std::chrono::high_resolution_clock::now();
        // const std::chrono::milliseconds timeout(30); // 30 milliseconds timeout

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
            
            tf2_ros::Buffer tfBuffer_temp(ros::Duration(1.0));

            tf2_ros::TransformListener tfListener_temp(tfBuffer_temp, nh);

            try {
            geometry_msgs::TransformStamped transformStamped =
                tfBuffer_temp.lookupTransform(targetFrame,
                                            sourceFrame,
                                            msg_storage->header.stamp,
                                            ros::Duration(1.0));

            Eigen::Isometry3d raw_trans = tf2::transformToEigen(transformStamped);

            Eigen::Matrix4f trans = raw_trans.matrix().cast<float>();

            pcl::transformPointCloud(*cloud_in, *cloud_out, trans);

            return cloud_out;

            } catch (tf2::TransformException &ex) {
                //ROS_INFO_STREAM("Encounted error in transforming frames: " + msg_storage->header.stamp + " Error raw: " + ex.what() + );
                return cloud_in;
            }



            

            // while(!tfBuffer_temp.canTransform(targetFrame, source, ros::Time::now(), &source)) {
            //     ROS_INFO_STREAM("Error:"+source+"\n\n");
            // };

            // pcl_ros::transformPointCloud( 	
            //     targetFrame,
		    //     *cloud_in,
		    //     *cloud_out,
		    //     tfListener 
	        // );

            
        
    }

    void printPCLPointCloud2ToConsole(const pcl::PCLPointCloud2 cloud2) {

        // Convert PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(cloud2, *cloudXYZ);

        if (cloudXYZ->empty()) {
            std::cerr << "Error: Converted PointCloud is empty." << std::endl;
            return;
        }

        std::cout << "(x, y) coordinates of points in the cloud:" << std::endl;

        for (const auto& point: cloudXYZ->points) {
            std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr beefUpCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double xTop, double xBot, double yTop, double yBot) {
        for (const auto& point : cloud->points) {
            if (point.x > xTop || point.x < xBot || point.y > yTop || point.y < yBot) {
                cloud->push_back(point);
            }
        }

        return cloud;
    }
};