#include "LidarFeatureExtractor/LidarFeatureExtractor.h"

typedef pcl::PointXYZINormal PointType;

ros::Publisher pubFullLaserCloud;
ros::Publisher pubSharpCloud;
ros::Publisher pubFlatCloud;
ros::Publisher pubNonFeature;

LidarFeatureExtractor* lidarFeatureExtractor;
pcl::PointCloud<PointType>::Ptr laserCloud;
pcl::PointCloud<PointType>::Ptr laserConerCloud;
pcl::PointCloud<PointType>::Ptr laserSurfCloud;
pcl::PointCloud<PointType>::Ptr laserNonFeatureCloud;
int Lidar_Type = 0;
int N_SCANS = 6;
bool Feature_Mode = false;
bool Use_seg = false;
tf::TransformListener* listener;

void lidarCallBackPc2(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr laser_cloud_custom(new pcl::PointCloud<pcl::PointXYZINormal>());

    pcl::fromROSMsg(*msg, *laser_cloud);

    for (uint64_t i = 0; i < laser_cloud->points.size(); i++)
    {
        auto p=laser_cloud->points.at(i);
        pcl::PointXYZINormal p_custom;
        if(Lidar_Type == 0||Lidar_Type == 1)
        {
            if(p.x < 0.01) continue;
        }
        else if(Lidar_Type == 2)
        {
            if(std::fabs(p.x) < 0.01) continue;
        }
        p_custom.x=p.x;
        p_custom.y=p.y;
        p_custom.z=p.z;
        p_custom.intensity=p.intensity;
        p_custom.normal_x=float (i)/float(laser_cloud->points.size());
        p_custom.normal_y=i%4;
        laser_cloud_custom->points.push_back(p_custom);
    }

    lidarFeatureExtractor->FeatureExtract_Mid(laser_cloud_custom, laserConerCloud, laserSurfCloud);

    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*laser_cloud_custom, laserCloudMsg);
    laserCloudMsg.header = msg->header;
    pubFullLaserCloud.publish(laserCloudMsg);

}

void lidarCallBackHesai(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PCLPointCloud2::Ptr laser_cloud(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr laser_cloud_custom(new pcl::PointCloud<pcl::PointXYZINormal>());
    sensor_msgs::PointCloud2Ptr transformed(new sensor_msgs::PointCloud2());

    pcl_ros::transformPointCloud("base_footprint", *msg, *transformed, *listener);
    pcl_conversions::toPCL(*transformed, *laser_cloud);

    const unsigned int pt_step = laser_cloud->point_step;
    std::uint8_t* data_ptr = laser_cloud->data.data();
    const int x_idx = pcl::getFieldIndex(*laser_cloud, "x");
    const int y_idx = pcl::getFieldIndex(*laser_cloud, "y");
    const int z_idx = pcl::getFieldIndex(*laser_cloud, "z");
    const int intensity_idx = pcl::getFieldIndex(*laser_cloud, "intensity");
    const int ring_idx = pcl::getFieldIndex(*laser_cloud, "ring");
    unsigned int x_offset = laser_cloud->fields[x_idx].offset;
    unsigned int y_offset = laser_cloud->fields[y_idx].offset;
    unsigned int z_offset = laser_cloud->fields[z_idx].offset;
    unsigned int intensity_offset = laser_cloud->fields[intensity_idx].offset;
    unsigned int ring_offset = laser_cloud->fields[ring_idx].offset;
    std::vector<unsigned> rings;
    rings.reserve(N_SCANS);
    for (unsigned i = 0; i < laser_cloud->width * laser_cloud->height; i++)
    {
      pcl::PointXYZINormal p_custom;
      p_custom.x = *reinterpret_cast<float *>(data_ptr + i * pt_step + x_offset);
      p_custom.y = *reinterpret_cast<float *>(data_ptr + i * pt_step + y_offset);
      p_custom.z = *reinterpret_cast<float *>(data_ptr + i * pt_step + z_offset);
      p_custom.intensity = *reinterpret_cast<float *>(data_ptr + i * pt_step + intensity_offset);
      p_custom.normal_x = float(i)/float(laser_cloud->width * laser_cloud->height);
      unsigned ring = *reinterpret_cast<unsigned *>(data_ptr + i * pt_step + ring_offset);
      auto it = std::find(rings.begin(), rings.end(), ring);
      p_custom.normal_y = it - rings.begin();
      if (it == rings.end()) { rings.push_back(ring); }
      laser_cloud_custom->points.push_back(p_custom);
    }

    lidarFeatureExtractor->FeatureExtract_Hesai(laser_cloud_custom, laserConerCloud, laserSurfCloud);

    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*laser_cloud_custom, laserCloudMsg);
    laserCloudMsg.header = transformed->header;
    pubFullLaserCloud.publish(laserCloudMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ScanRegistration");
  ros::NodeHandle nodeHandler("~");

  ros::Subscriber customCloud,pc2Cloud;

  std::string config_file;
  int msg_type=0;
  nodeHandler.getParam("config_file", config_file);
  nodeHandler.getParam("msg_type", msg_type);

  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cout << "config_file error: cannot open " << config_file << std::endl;
    return false;
  }
  Lidar_Type = static_cast<int>(fsSettings["Lidar_Type"]);
  N_SCANS = static_cast<int>(fsSettings["Used_Line"]);
  Feature_Mode = static_cast<int>(fsSettings["Feature_Mode"]);
  Use_seg = static_cast<int>(fsSettings["Use_seg"]);

  int NumCurvSize = static_cast<int>(fsSettings["NumCurvSize"]);
  float DistanceFaraway = static_cast<float>(fsSettings["DistanceFaraway"]);
  int NumFlat = static_cast<int>(fsSettings["NumFlat"]);
  int PartNum = static_cast<int>(fsSettings["PartNum"]);
  float FlatThreshold = static_cast<float>(fsSettings["FlatThreshold"]);
  float BreakCornerDis = static_cast<float>(fsSettings["BreakCornerDis"]);
  float LidarNearestDis = static_cast<float>(fsSettings["LidarNearestDis"]);
  float KdTreeCornerOutlierDis = static_cast<float>(fsSettings["KdTreeCornerOutlierDis"]);
  listener = new tf::TransformListener();


  laserCloud.reset(new pcl::PointCloud<PointType>);
  laserConerCloud.reset(new pcl::PointCloud<PointType>);
  laserSurfCloud.reset(new pcl::PointCloud<PointType>);
  laserNonFeatureCloud.reset(new pcl::PointCloud<PointType>);

  if (Lidar_Type == 0)
  {
  }
  else if (Lidar_Type == 1)
  {
  }
  else if(Lidar_Type==2){
      if (msg_type==0)
          ;
      else if(msg_type==1)
          pc2Cloud=nodeHandler.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 100, &lidarCallBackPc2);
  }
  else if (Lidar_Type == 3)
  {
    customCloud = nodeHandler.subscribe<sensor_msgs::PointCloud2>("/lio_lidar_in", 100, &lidarCallBackHesai);
  }
  pubFullLaserCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_full_cloud", 10);
  pubSharpCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_less_sharp_cloud", 10);
  pubFlatCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_less_flat_cloud", 10);
  pubNonFeature = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_nonfeature_cloud", 10);

  lidarFeatureExtractor = new LidarFeatureExtractor(N_SCANS,NumCurvSize,DistanceFaraway,NumFlat,PartNum,
                                                    FlatThreshold,BreakCornerDis,LidarNearestDis,KdTreeCornerOutlierDis);

  ros::spin();

  return 0;
}

