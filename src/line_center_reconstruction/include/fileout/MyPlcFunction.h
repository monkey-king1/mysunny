#ifndef MYPLCFUNCTION_H
#define MYPLCFUNCTION_H

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/range_image_visualizer.h> //深度图像可视化

#define ROWS_PROPORTION              1.0     //相机图像高度比例   实际距离(mm)/相机像素距离
#define COLS_PROPORTION              1.0     //相机图像宽度比例   实际距离(mm)/相机像素距离

#define DEEPIMG_CALLBACKNUM_DNUM     5     //采集深度图时多采集的帧数

#define CLOULD_POINT_NOTDATE        0   //深度值不存在时点云的值


class MyPlcFunction
{
public:
    MyPlcFunction();
    ~MyPlcFunction();
    
    void cvpoint3f_to_oneline_pclclould(std::vector<cv::Point3f>cv_cloud,float x,pcl::PointCloud<pcl::PointXYZRGB>::Ptr *point_cloud_ptr_Out);//把cv_cloud数组输出为一行点云

    void float_to_oneline_pclclould(float *f_data,int f_datanum,float y,pcl::PointCloud<pcl::PointXYZRGB>::Ptr *point_cloud_ptr_Out);//把f_data数组输出为一行点云

    void updata_color_pclclould(pcl::PointCloud<pcl::PointXYZRGB>::Ptr *point_cloud_ptr_In,pcl::PointCloud<pcl::PointXYZRGB>::Ptr *point_cloud_ptr_Out);//把点云重新按Z轴刷新颜色

    void pclclould_to_rangeImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr *point_cloud_ptr_In,pcl::RangeImage *rangeImage); //把point_cloud_ptr点云变为深度图rangeImage

    void cv_f32deepimg_to_show8deepimg(cv::Mat f32_deepimg,cv::Mat *f8_deepimg);  //把浮点型深度图转成8位图，死区用0表示

    void pointCloud2imgI(pcl::PointCloud<pcl::PointXYZRGB>::Ptr *point_cloud_ptr_In, cv::Mat *f8_deepimg,double resolution);//把点云投影到Z平面，强度用Z表示

    int32_t Myfixfdata(float *dataIn_Out,u_int8_t *mask,int32_t num);

    void addpoint_image(cv::Mat *f8_deepimg,int coldis,int rowdis);//插补
};

#endif // MYPLCFUNCTION_H
