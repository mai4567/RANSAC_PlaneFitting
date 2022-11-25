#include "planefitting.h"

DL_Plane::DL_Plane(QObject *parent) : QObject(parent)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonplane (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ = cloud;
    plane_ = plane;
    nonplane_ = nonplane;
}


//设置输入点云
void DL_Plane::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    cloud_ = cloud;
}


//求解点到平面距离(使用前要更新平面参数)
double DL_Plane::getDistance(pcl::PointXYZ point){
    double up = std::abs(A*point.x+B*point.y+C*point.z+1);
    double down = std::sqrt(std::pow(A,2)+std::pow(B,2)+std::pow(C,2));
    double dis = up/down;
    return dis;
}


//平面拟合算法
Eigen::Vector3d DL_Plane::getFlat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    Eigen::Matrix3d rot;
    double x2,xy,x,y2,y,zx,zy,z;
    for(int i=0;i<cloud->points.size();i++){
        x2 += cloud->points[i].x * cloud->points[i].x;
        xy += cloud->points[i].x * cloud->points[i].y;
        x += cloud->points[i].x;
        y2 += cloud->points[i].y * cloud->points[i].y;
        y += cloud->points[i].y;
        zx += cloud->points[i].x * cloud->points[i].z;
        zy += cloud->points[i].y * cloud->points[i].z;
        z += cloud->points[i].z;
    }
    //为矩阵赋值
    rot<<x2,  xy,  x,
         xy,  y2,  y,
         x,   y,   cloud->points.size();
    //为列向量赋值
    Eigen::Vector3d eq(zx,zy,z);
    Eigen::Vector3d X = rot.colPivHouseholderQr().solve(eq);
//    std::cout<<X<<std::endl;
//    std::cout<<X[0]<<" "<<X[1]<<" "<<X[2]<<std::endl;
    return X;
}


//更新平面参数
void DL_Plane::updateParameter(Eigen::Vector3d x){
    a0 = x[0];
    a1 = x[1];
    a2 = x[2];
    A=a0/a2;
    B=a1/a2;
    C=-1/a2;
}


//求平面内点
void DL_Plane::planeFitting(double dis){
    plane_->clear();
    for (int i=0;i<cloud_->points.size();i++){
        if(getDistance(cloud_->points[i])<dis){
             plane_->points.push_back(cloud_->points[i]);
        }
        else{
            nonplane_->points.push_back(cloud_->points[i]);
            //std::cout<<"dis:"<<getDistance(cloud->points[i])<<std::endl;
        }
    }
}


//取平面点云
pcl::PointCloud<pcl::PointXYZ>::Ptr DL_Plane::getPlanePoints(void){
    return plane_;
}


//设置距离参数
void DL_Plane::setParameter(double dis){
    distance = dis;
}


//平面拟合处理
void DL_Plane::planeProcess(void){
    Eigen::Vector3d result = getFlat(this->cloud_);
    updateParameter(result);
    planeFitting(this->distance);
}


//点云可视化
void DL_Plane::showCloud(std::string windowname,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (windowname));
    viewer->setBackgroundColor (0.5, 0.5, 0.5);  //设置背景
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");  //显示点云
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");  //设置点尺寸
    viewer->addCoordinateSystem (100.0);  //设置坐标轴尺寸
//    while (!viewer->wasStopped ())
//    {
//      viewer->spinOnce (100);
//      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }
    //cout<<"Point couting in "<<windowname<<": "<<cloud->size()<<endl;
}


DL_PlaneFitting::DL_PlaneFitting(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr restpoints(new pcl::PointCloud<pcl::PointXYZ>);
    restpoints_ = restpoints;
}


//更新平面参数
void DL_PlaneFitting::updatePlaneParameter(DL_Plane *dl_plane, Eigen::Vector3d x){
    dl_plane->updateParameter(x);
}


//随机生成三个点，存在flatpoints中
void DL_PlaneFitting::randomGenerate(pcl::PointCloud<pcl::PointXYZ>::Ptr restpoints,int p_num){
    // 生成随机的三个点拟合平面
    LARGE_INTEGER seed;
    QueryPerformanceFrequency(&seed);
    QueryPerformanceCounter(&seed);
    srand(seed.QuadPart);
    pcl::PointCloud<pcl::PointXYZ>::Ptr flatpoints(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0;i<p_num;i++){
        int index1 = rand()%restpoints->points.size();
        flatpoints->points.push_back(restpoints->points[index1]);
//        std::cout<<" "<<i<<":"<<index1;
    }
//    std::cout<<std::endl;
    flatpoints_ = flatpoints;
}


//RANSAC法拟合平面
void DL_PlaneFitting::ransacPlaneFitting(){
    //记录原点云数量
    int total_num = cloud_->points.size();
    std::cout<<"原点云总数："<<total_num<<std::endl;
    //存放结果
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
    //存放剩余的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr restpoints(new pcl::PointCloud<pcl::PointXYZ>);
    restpoints->points = cloud_->points;

    //存放每次迭代的点云
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> max_Planes;
    //存放每次迭代剩余的点云
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> rest_Planes;
    int N=0;
    while(restpoints->points.size()>stop_rate*total_num){
        N+=1;
        max_Planes.clear();
        rest_Planes.clear();
        //设置每层迭代次数
        int n=iteration_time;
        while(--n>0){
            randomGenerate(restpoints,3);
            DL_Plane dl_plane;
            dl_plane.setInputCloud(restpoints);
            Eigen::Vector3d result = dl_plane.getFlat(flatpoints_);
            dl_plane.updateParameter(result);
            //设置拟合平面点的距离阈值
            dl_plane.planeFitting(dis_threshold);
            //如果拟合平面的内点数量符合要求
            if(dl_plane.plane_->points.size()>min_rate*total_num){
                max_Planes.push_back(dl_plane.plane_);
                rest_Planes.push_back(dl_plane.nonplane_);
            }
        }
        //提取最大平面作为其中一个结果
        int max_num=0;
        int index = -1;
        for (int i=0;i<max_Planes.size();i++){
            if (max_Planes[i]->points.size()>max_num){
                max_num = max_Planes[i]->points.size();
                index = i;
            }
        }
        result.push_back(max_Planes[index]);
        //重新设置剩余的点
        restpoints->points = rest_Planes[index]->points;
        //std::cout<<"剩余的点的数量"<<restpoints->points.size()<<std::endl;
        std::cout<<"第"<<N<<"次迭代比例为:"<<double((restpoints->points.size()+0.01)/total_num)<<std::endl;
    }
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("RANSAC法拟合平面点云"));
    viewer->setBackgroundColor (0.5, 0.5, 0.5);  //设置背景
    viewer->addCoordinateSystem (100.0);  //设置坐标轴尺寸
    for(int i=0;i<result.size();i++){
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(result[i]);
        viewer->addPointCloud<pcl::PointXYZ> (result[i], rgb,"sample cloud"+std::to_string(i));  //显示点云
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"+std::to_string(i));  //设置点尺寸
    }
}


//线程入口函数
void DL_PlaneFitting::planeThreadFitting(int n){  //分线程拟合平面n为每个线程需要迭代的次数
//    std::cout<<"线程"<<std::this_thread::get_id()<<"开始执行..."<<std::endl;
    while(--n>0){
        randomGenerate(restpoints_,3);
        DL_Plane dl_plane;
        dl_plane.setInputCloud(restpoints_);
        Eigen::Vector3d result = dl_plane.getFlat(flatpoints_);
        dl_plane.updateParameter(result);
        //设置拟合平面点的距离阈值
        dl_plane.planeFitting(dis_threshold);
        //如果拟合平面的内点数量符合要求
        if(dl_plane.plane_->points.size()>min_rate*total_num_){
            //加锁，互斥量
            my_mutex.lock();
            max_Planes_.push_back(dl_plane.plane_);
            rest_Planes_.push_back(dl_plane.nonplane_);  //将内点与外点保存
            my_mutex.unlock();
        }
    }
//    std::cout<<"线程"<<std::this_thread::get_id()<<"执行完毕..."<<std::endl;
}


//RANSAC法拟合平面(多线程版本)
void DL_PlaneFitting::ransacPlaneFittingThread(){
    //记录原点云数量
    total_num_ = cloud_->points.size();
    std::cout<<"原点云总数："<<total_num_<<std::endl;
    //存放结果
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
    //存放剩余的点
    restpoints_->points = cloud_->points;
    int N=0;
    while(restpoints_->points.size()>stop_rate*total_num_){
        N+=1;
        //每次迭代前将类变量清除
        max_Planes_.clear();
        rest_Planes_.clear();
        mythreads_.clear();
        //设置每层迭代次数
        int n=iteration_time;
        //多线程迭代求平面
        for (int i=0;i<core_;i++){
            mythreads_.push_back(std::thread(&DL_PlaneFitting::planeThreadFitting,this,int(n/core_)));
//            cout<<"线程"<<i<<"已启动"<<std::endl;
        }
        //线程与主线程汇合
        for (auto iter = mythreads_.begin();iter<mythreads_.end();iter++){
            iter->join();
        }

        //提取最大平面作为其中一个结果
        int max_num=0;
        int index = -1;
        for (int i=0;i<max_Planes_.size();i++){
            if (max_Planes_[i]->points.size()>max_num){
                max_num = max_Planes_[i]->points.size();
                index = i;
            }
        }
        result.push_back(max_Planes_[index]);
        //重新设置剩余的点
        restpoints_->points = rest_Planes_[index]->points;
        //std::cout<<"剩余的点的数量"<<restpoints_->points.size()<<std::endl;
        std::cout<<"第"<<N<<"次迭代比例为:"<<double((restpoints_->points.size()+0.01)/total_num_)<<std::endl;
    }
    std::cout<<"迭代完成"<<std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("RANSAC法拟合平面点云"));
    viewer->setBackgroundColor (0.5, 0.5, 0.5);  //设置背景
    viewer->addCoordinateSystem (100.0);  //设置坐标轴尺寸
    for(int i=0;i<result.size();i++){
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(result[i]);
        viewer->addPointCloud<pcl::PointXYZ> (result[i], rgb,"sample cloud"+std::to_string(i));  //显示点云
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"+std::to_string(i));  //设置点尺寸
    }
}
