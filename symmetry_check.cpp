#include <../ProstheticEye/symmetry.hpp>

typedef pcl::visualization::PCLVisualizer Visualizer;
typedef std::tuple<float,float,float> plane;

boost::shared_ptr<Visualizer> makeVisualizer(){
	boost::shared_ptr<Visualizer> viewer (new Visualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters ();
	return (viewer);
}

void addCloud(boost::shared_ptr<Visualizer> viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,const std::string &label){
	viewer->addPointCloud<pcl::PointXYZ> (cloud, label.c_str());
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, label.c_str());
}

void addRedCloud(boost::shared_ptr<Visualizer> viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,const std::string &label){
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, label.c_str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, label.c_str());
}

float planeMaker(const float x,const float y,const plane &p){
	const float a = std::get<0>(p);
	const float b = std::get<1>(p);
	const float c = std::get<2>(p);
	return (a*a+b*b+c*c - a*x - b*y)/c;
}

int main(){
	// pcl::PointXYZ p1(1,1,1);
	// pcl::PointXYZ p2(3,3,3);
	// symmetric_pair a(p1,p2,0,1);
	// std::cout<<a.alpha<<" "<<a.beta<<" "<<a.gamma<<std::endl;
	// exit(0);
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	readfsamp("../ProstheticEye/Scan2.xyz",cloud);
	auto v = makeVisualizer();
	addCloud(v,cloud,"cloud");
	//plane p(0.032072,0.031865,0.129676);
	plane p(0.031160,-0.006922,-0.032507);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	for(int i=0;i<1000;i++){
		float x = ((float)(rand()%3000))/10.0 - 100.0;
		float y = ((float)(rand()%3000))/10.0 - 100.0;
		float z = planeMaker(x,y,p);
		plane_cloud->points.push_back(pcl::PointXYZ(x,y,z));
	}
	addRedCloud(v,plane_cloud,"plane1");
	while(!v->wasStopped()){
		v->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}