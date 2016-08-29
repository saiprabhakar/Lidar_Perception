#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <unistd.h>
#include <iostream>
#include <pcl/io/io.h>

#include <pcl/filters/extract_indices.h>
#include <string>

ros::Publisher pub;
//new_branch

void PcltoROSMSG(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, sensor_msgs::PointCloud2& rosMsg) {
  //function to convert Poincloud<pointXYZ> to pointcloud2 for some reason the inbuild code doesnt work
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*pclCloud, pcl_pc2);
  pcl_conversions::moveFromPCL(pcl_pc2, rosMsg);
}


////////////////////////////////////////////////////////////////////////

class MySubscriber
{
  std::string po_topic_name;//1
  ros::Subscriber po_sub;
  float filter_radius;//2
  int filter_neighbour;//3
    
  bool filter_voxel;
  
  float filter_voxel_size;//4
  
  bool do_region_flow_cluster;
  
  int cluster_min_size;//5 = 50;
  int cluster_max_size;//6 =100;
  float normal_search_radius;//7 =.03;
  int normal_search_neigh;//8 =50;
  float SmoothnessThreshold;//9 =3.0;
  float CurvatureThreshold;//10 =1.0;
  int reg_Neighbours;//11 =30;  
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_final;

  float Ransac_dist_thres;
  bool updated;

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
  void region_flow_cluster (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);//, pcl::visualization::CloudViewer& viewer);
  //void fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void fitPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_p, pcl::ModelCoefficients::Ptr& coefficients);
  
public:

  MySubscriber( ros::NodeHandle ao_nh, int no_arg, char** arguments );
  
};

////////////////////////////////////////////////////////////////////////
int num_arguments= 6;
MySubscriber::MySubscriber( ros::NodeHandle ao_nh, int no_arg, char** arguments )
{
  cout<<"num of arg "<<no_arg<<"\n";
  if(no_arg< num_arguments) {
    cout<<"\n usage arguments : 1:po_topic_name,  2:float:filter_radius,  3:filter_neighbour,    4:float:filter_voxel_size, 5:float:Ransac_dist_thres";
    //cout<<"  5:cluster_min_size,  6:cluster_max_size,  7:float:normal_search_radius,";
    //cout<<"  8:normal_search_neigh,   9:float:SmoothnessThreshold,   10:float:CurvatureThreshold,";
    //cout<<"  11:reg_Neighbours\n\n";
    
  } else {

    po_topic_name= arguments[1];
    filter_radius= atof (arguments[2]);
    filter_neighbour = atoi (arguments[3]);
    filter_voxel_size= atof (arguments[4]);
    Ransac_dist_thres= atof (arguments[5]);
    //cluster_min_size= atoi (arguments[5]);
    //cluster_max_size= atoi (arguments[6]);
    //normal_search_radius= atof (arguments[7]);
    //normal_search_neigh= atoi (arguments[8]);
    //SmoothnessThreshold= atof (arguments[9]);
    //CurvatureThreshold= atof (arguments[10]);
    //reg_Neighbours= atoi (arguments[11]);
    
    do_region_flow_cluster=false;
    updated=false;
    filter_voxel=false;
    if(filter_voxel_size>0)
      filter_voxel=true;
    
    
    po_sub = ao_nh.subscribe (po_topic_name, 1, &MySubscriber::cloud_cb, this );
    pub = ao_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    // Spin
    //ros::spin ();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_final_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_filtered_final_ptr= cloud_filtered_final.makeShared();
    
    //viewer setup
    pcl::visualization::PCLVisualizer viewer ("Point Cloud"); 
    viewer.addCoordinateSystem(2.0);
    
    while (!viewer.wasStopped ())
    {
      ros::spinOnce();
      if (updated) {
        updated=false;
        cloud_filtered_final_ptr= cloud_filtered_final.makeShared();
        
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
        int i = 0, nr_points = (int) cloud_filtered_final_ptr->points.size ();
        while (cloud_filtered_final_ptr->points.size () > 0.1 * nr_points)
        {
          i++;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
          //std::cerr << "PointCloud size: " << cloud_filtered_final_ptr->width * cloud_filtered_final_ptr->height << " data points." << std::endl;
          fitPlane(cloud_filtered_final_ptr, cloud_p, coefficients);
          //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> target_color (cloud_p, 0, std::max(255- i*(255/(4+1)),0), std::max(255- i*(255/(4+1)),0)); 
          //std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      //<< coefficients->values[1] << " "
                                      //<< coefficients->values[2] << " " 
                                      //<< coefficients->values[3] << std::endl;
          //std::cout<< "Size 9999 of inliners: "<< cloud_p->width*cloud_p->height<< std::endl;
          int r, b, g;
          switch (i) {
            case 1:
              r= 0;
              b= 255;
              g= 0;
              break;
            case 2:
              r= 0;
              b= 255;
              g= 255;
              break;
            case 3:
              r= 0;
              b= 0;
              g= 255;
              break;
            default :
              r= 0;
              b= std::max(255- (i-3)*(255/(4+1)),0);
              g= std::max(255- (i-3)*(255/(4+1)),0);
              break;
          }
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> target_color (cloud_p,r,g,b); 
          viewer.addPointCloud(cloud_p, target_color, "ptCloud_"+boost::lexical_cast<std::string>(i));   
          float m1= (coefficients->values[0]);
          float m2= (coefficients->values[1]);
          float m3= (coefficients->values[2]);
          float c= (coefficients->values[3]);
          float temp= std::sqrt( m1*m1 + m2*m2 + m3*m3);
          float distToOrigin= fabs(c)/temp;
          double ptCloseToOrigin[3]= { m1*distToOrigin/temp, m2*distToOrigin/temp, m3*distToOrigin/temp};
          double theta1= atan(m1/m2)*(180+0.1f)/3.1457;
          double theta2= atan(m1/m3)*(180+0.1f)/3.1457;
          double theta3= atan(m3/m2)*(180+0.1f)/3.1457;
          viewer.addPlane( *coefficients, ptCloseToOrigin[0], ptCloseToOrigin[1], ptCloseToOrigin[2], "plane_"+boost::lexical_cast<std::string>(i) );
          //cout<<" equation "<<m1<<"*x + "<<m2<<"*y + "<<m3<<"*z + "<<c<<" at \t"<<distToOrigin<<"\n";
          cout<<"theta1 "<< theta1<< " theta2 "<<theta2<< " theta3 "<<theta3<<"\n";
          
          int vehicleFound= 0;
          if ( fabs(theta3)>70 ) {
            //std::cout<<" z axis\n";
            cout<<" equation "<<m1<<"*x + "<<m2<<"*y + "<<m3<<"*z + "<<c<<" at \t"<<distToOrigin<<"\n";
            std::cout<<"vehicle found \n";
            vehicleFound=1;
            //find a verticle plane
          } //else if (fabs(theta1)<10 && fabs(theta2)<10 && fabs(theta3)<10) {
            //std::cout<<" y axis\n";
          //} else {
            //std::cout<<"bad segmentation\n";
          //}
          
          if(i>=6 || vehicleFound== 1) {
            break;
            //sleep(0.5);
          }
          //std::cerr << "Original PointCloud size: " << cloud_filtered_final.width * cloud_filtered_final.height << " data points." << std::endl;
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> target_color (cloud_filtered_final_ptr, 255, 0, 0); 
        viewer.addPointCloud(cloud_filtered_final_ptr, target_color, "remaining");
        //cout<<"displaying "<<cloud_filtered_final_ptr->width<<" "<<cloud_filtered_final_ptr->height<<"\n";

        //viewer.updatePointCloud(cloud_filtered_final_ptr, green_target);
        viewer.spinOnce (1);
        cout<<"------------------------new loop\n\n";
        //while(1);
      }
    }
    
  }
}
////////////////////////////////////////////////////////////////////////

void MySubscriber::fitPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_p, pcl::ModelCoefficients::Ptr& coefficients) {
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (Ransac_dist_thres);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0) {
    cout<<"Could not estimate a planar model for the given dataset.";
    //return (-1);
  }
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  
  extract.setNegative (true);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.filter (*cloud_f);
  cloud.swap (cloud_f);
}
////////////////////////////////////////////////////////////////////////
void MySubscriber::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //intensity data getting lost
  updated=true;
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  std::vector< int > a;
  pcl::removeNaNFromPointCloud (cloud,cloud,a);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered=cloud.makeShared();

  std::cout<<"before voxel filtering "<<cloud_filtered->width*cloud_filtered->height<<"\n";
  // voxel filtering
  if(filter_voxel) {
    pcl::VoxelGrid<pcl::PointXYZ >sor;
    sor.setInputCloud (cloud_filtered);
    sor.setLeafSize (filter_voxel_size, filter_voxel_size, filter_voxel_size);
    sor.filter (*cloud_filtered);
  } 
  
  //clustering
  if(do_region_flow_cluster)
    region_flow_cluster (cloud_filtered);

  // radius filtering
  std::cout<<"before radius filtering "<<cloud_filtered->width*cloud_filtered->height<<"\n";
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud (cloud_filtered);
  outrem.setRadiusSearch (filter_radius);
  outrem.setMinNeighborsInRadius (filter_neighbour);
  outrem.filter (*cloud_filtered);
  std::cout<<"after radius filtering "<<cloud_filtered->width*cloud_filtered->height<<"\n";
  
  // Publish the data
  sensor_msgs::PointCloud2 output;
  PcltoROSMSG(cloud_filtered,output);
  
  //cloud_filtered_final=cloud_filtered;
  copyPointCloud(*cloud_filtered, cloud_filtered_final);
  //cout<<"size "<<cloud.width<<" "<<cloud.height<<"\n";
  pub.publish (output);
}

////////////////////////////////////////////////////////////////////////

void MySubscriber::region_flow_cluster (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  //normal_estimator.setKSearch (normal_search_neigh);
  normal_estimator.setRadiusSearch (normal_search_radius);
  normal_estimator.compute (*normals);

  //pcl::IndicesPtr indices (new std::vector <int>);
  //pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (cluster_min_size);
  reg.setMaxClusterSize (cluster_max_size);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (reg_Neighbours);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (SmoothnessThreshold / 180.0 * M_PI);
  reg.setCurvatureThreshold (CurvatureThreshold);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  //int counter = 0;
  for(int i=0; i<clusters.size(); i++)
  { 
    std::cout << "cluster has " << clusters[i].indices.size () <<std::endl;
  }
  std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  //return (0);  
  sleep(1);
}


////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  MySubscriber s(nh,argc, argv);

  
}
