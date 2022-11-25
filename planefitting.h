#ifndef PLANEFITTING_H
#define PLANEFITTING_H

#include <QObject>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<Eigen/Dense>
#include <random>
#include <windows.h>
//多线程头文件
#include <thread>
#include <mutex>


//平面拟合类
class DL_Plane : public QObject
{
    Q_OBJECT
public:
    explicit DL_Plane(QObject *parent = nullptr);
    /*https://blog.csdn.net/konglingshneg/article/details/82585868
      设平面方程为ax+by+cz+d=0
      两边同时除d则有(a/d)X+(b/d)Y+(c/d)z+1=0
      令a0=-a/c;a1=-b/c;a2=-d/c 即（a0/a2）X+（a1/a2）Y +（-1/a2）Z+1=0
    最后直线方程写成AX+BY+CZ+1=0*/

    //平面表示参数
    double A;
    double B;
    double C;
    //距离阈值
    double distance = 1.0;
    //拟合点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    //平面内点
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_;
    //平面外点
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonplane_;

    //平面拟合
    Eigen::Vector3d getFlat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    //设置输入点云
    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    //更新平面参数
    void updateParameter(Eigen::Vector3d x);
    //求点到平面距离
    double getDistance(pcl::PointXYZ point);
    //求平面点云
    void planeFitting(double dis);
    //取平面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPlanePoints(void);
    //设置参数
    void setParameter(double dis);
    //平面拟合处理
    void planeProcess(void);
    //点云可视化
    void showCloud(std::string windowname,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
private:
    //拟合平面参数
    double a0;
    double a1;
    double a2;
signals:

public slots:
};


//RANSAC法拟合类
class DL_PlaneFitting : public DL_Plane
{
public:

    //总点的数量
    int all_pointsnum;
    //内点点集
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_points_;
    //最终输出
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> result_;
    //对应输出点集的int列表
    std::vector<int> max_output;
    //多线程版本参数
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> rest_Planes_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> max_Planes_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr restpoints_;
    int total_num_;
    std::mutex my_mutex;
    int core_;
    std::vector<std::thread> mythreads_;  //容器管理多个线程


    DL_PlaneFitting();
    void updatePlaneParameter(DL_Plane *dl_plane,Eigen::Vector3d x);
    //参数设置
    void setIterationTime(int num){iteration_time = num;}
    void setDisThreshold(double dis){dis_threshold = dis;}
    void setMinRate(double rate){min_rate = rate;}
    void setStopRate(double rate){stop_rate = rate;}
    //RANSAC法拟合平面
    void ransacPlaneFitting();
    void getResultPlanes();
    //多线程方法
    void planeThreadFitting(int n);  //线程入口函数
    void ransacPlaneFittingThread();  //多线程版本

private:
    double dis;
    //迭代次数
    int iteration_time = 1000;
    //距离阈值
    double dis_threshold = 2.0;
    //平面点数量与点云总数最小比例
    double min_rate = 0.05;
    //终止条件
    double stop_rate = 0.25;
    //随机生成的三个假平面点
    pcl::PointCloud<pcl::PointXYZ>::Ptr flatpoints_;
    //从点云中随机生成三个点
    void randomGenerate(pcl::PointCloud<pcl::PointXYZ>::Ptr restpoints,int p_num);
};

#endif // PLANEFITTING_H
