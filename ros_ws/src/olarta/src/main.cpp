#define DEBUG 1
#include "main.hpp"
#include <iostream>
#include <thread>
#include <chrono>
//BAG_FILE_STR=sim_nowalls.bag docker compose --profile playback --profile visualization --profile olarta build

Node::Node() : nh{},
        tfListener(nh, ros::Duration(1), true), //change
        map(new pcl::PointCloud<pcl::PointXYZ>),
        s_scan(nh.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 10, &Node::scanCallback, this)),
        s_map(nh.subscribe("/map", 10, &Node::mapCallback, this)),
        p_voxelPC2(nh.advertise<pcl::PCLPointCloud2>("/voxel_pc", 1)),
        p_adjustedScan(nh.advertise<pcl::PCLPointCloud2>("/adjust_pc", 1)),
        p_PCMap(nh.advertise<pcl::PCLPointCloud2>("/PCMap", 1)),
        cumulativeTransform(Eigen::Matrix4f::Identity())
{
    std::cout << 
    "Launching \"The one Localization Algorithem to rule them all\""
     << std::endl;
}

Node::~Node() {
    std::cout << "Byeeeeeeeeeee" << std::endl;
}


void Node::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    msg_storage = msg;
    scanBuffer = scanToPC2(msg);
    scanBuffer = filterPC2(scanBuffer);
    pcl::PCLPointCloud2 mapPC2;
    pcl::toPCLPointCloud2(*map, mapPC2);
    publishPC2(p_PCMap, mapPC2);

    ICPCalc(scanBuffer);
}

void Node::mapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg) {
    map = GridToPC2(map_msg);
    
}

void Node::ICPCalc(pcl::PCLPointCloud2 cloudPeram) {

    //Lazer scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(cloudPeram, *cloudIn);

    pcl::PointCloud<pcl::PointXYZ>::Ptr storage(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    // //Look at this wacky thing
    // //Needed because sometimes tf screws up and gives empty cloud
    // do {
    //     temp = cloudBasic;
    //     //Transform cloud to targetFrame (odom)
    //     cloudIn = transformCloud(temp);
    // } while (cloudIn->empty());
    storage = cloudIn;
    cloudIn = transformCloud(cloudIn);

    //Just make it stop to not waste time on ICP
    if(storage = cloudIn) {
        return;
    }

    if (cloudIn->empty()) {
        ROS_INFO_STREAM("going to error");
    }

    //Transform with cumulative data
    pcl::transformPointCloud(*cloudIn, *cloudIn, cumulativeTransform);

    //Filters for statictical outliers
    cloudIn = filterStat(cloudIn);

    //Publish scan /w voxel filter
    pcl::PCLPointCloud2 voxelPub;
    pcl::toPCLPointCloud2(*cloudIn, voxelPub);
    publishPC2(p_voxelPC2, voxelPub);

    //Adds exta points to area near reefs
    //(4.64, 5.525), (13.18, 5.525)
    cloudIn = beefUpCloud(cloudIn, 4.64+1.2, 4.64-1.2, 5.525+1.2, 5.525-1.2);
    cloudIn = beefUpCloud(cloudIn, 13.18+1.2, 13.18-1.2, 5.525+1.2, 5.525-1.2);


    //Makes, passes parameters, and executes ICP algorithem
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloudIn);
	icp.setInputTarget(map);
	icp.setMaximumIterations(maxIterations); //Can work as low as 5
	icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance); //ignores points above 100cm away
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

    #ifdef DEBUG
    std::cout << "Matrix:" << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    std::cout << "ICP Confidence: " << icp.getFitnessScore() << std::endl;
    #endif

    //Needs to be looked into, simetimes the transform by TransformCloud functoin
    //is verry incorrect and FitnessScore goes to like 1.797...+e308
    if (icp.getFitnessScore() > 100) {
        return;
    }

    //Publishes transformed scan
    pcl::PCLPointCloud2 FincalPC2;
    pcl::toPCLPointCloud2(Final, FincalPC2);
    publishPC2(p_adjustedScan, FincalPC2);
    //printPCLPointCloud2ToConsole(FincalPC2); //used for beefUpCloud


    //Calculates new X & Y transform
    Eigen::Matrix4f positionTransform = icp.getFinalTransformation();
    positionTransform(0, 3) += cumulativeTransform(0, 3);
    positionTransform(1, 3) += cumulativeTransform(1, 3);
    positionTransform(2, 3) = 0; //2d

    //Calculates Rotation Transformation
    Eigen::Matrix4f currentTransform = icp.getFinalTransformation();
    currentTransform.col(3).setZero();
    cumulativeTransform *= currentTransform;

    //Reatatch Position Information
    cumulativeTransform.col(3) = positionTransform.col(3);	

    // 1. Extract Rotation and Translation:
    Eigen::Matrix3f rotation = cumulativeTransform.block<3, 3>(0, 0); // Top-left 3x3 is rotation
    Eigen::Vector3f translation = cumulativeTransform.block<3, 1>(0, 3); // Rightmost column is translation

    // 2. Convert to tf types:
    tf::Matrix3x3 tfRotation(rotation(0, 0), rotation(0, 1), rotation(0, 2),
                            rotation(1, 0), rotation(1, 1), rotation(1, 2),
                            rotation(2, 0), rotation(2, 1), rotation(2, 2));

    tf::Vector3 tfTranslation(translation(0), translation(1), translation(2));

    // 3. Create the tf::Transform:
    tf::Transform tftransform(tfRotation, tfTranslation);

    //Publishes transfrom from odom to odom_corrected
    broadcaster.sendTransform(
        tf::StampedTransform(tftransform, ros::Time::now(), targetFrame, "odom_corrected"));

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "olarta"); //Initilises this node
    Node MyNode;  //Sets up class
    ros::spin(); //needed
    return 0;

}