#include "laser_imagepos/laser_imagepos.hpp"

#include "opencv2/opencv.hpp"
namespace laser_imagepos
{

using rcl_interfaces::msg::SetParametersResult;

void LaserImagePos::alg108_declare_parameters()
{
    this->declare_parameter("als108_exposure_time", pm.als108_exposure_time);
    this->declare_parameter("als108_center_x", pm.als108_center_x);
    this->declare_parameter("als108_center_y", pm.als108_center_y);
    this->declare_parameter("als108_search_w", pm.als108_search_w);
    this->declare_parameter("als108_search_h", pm.als108_search_h);
    this->declare_parameter("als108_STC_alpha", pm.als108_STC_alpha);
    this->declare_parameter("als108_STC_beta", pm.als108_STC_beta);
    this->declare_parameter("als108_STC_rho", pm.als108_STC_rho);
    this->declare_parameter("als108_STC_sigma", pm.als108_STC_sigma);
}

void LaserImagePos::alg108_update_parameters()
{
  const auto & vp = this->get_parameters(KEYS_ALS108);
  for (const auto & p : vp) {
    if (p.get_name() == "als108_exposure_time") {
      pm.als108_exposure_time = p.as_int();
    } else if (p.get_name() == "als108_center_x") {
      pm.als108_center_x = p.as_int();
    } else if (p.get_name() == "als108_center_y") {
      pm.als108_center_y = p.as_int();
    }else if (p.get_name() == "als108_search_w") {
      pm.als108_search_w = p.as_int();
    } else if (p.get_name() == "als108_search_h") {
      pm.als108_search_h = p.as_int();
    } else if (p.get_name() == "als108_STC_alpha") {
      pm.als108_STC_alpha = p.as_int();
    } else if (p.get_name() == "als108_STC_beta") {
      pm.als108_STC_beta = p.as_int();
    } else if (p.get_name() == "als108_STC_rho") {
      pm.als108_STC_rho = p.as_int();
    } else if (p.get_name() == "als108_STC_sigma") {
      pm.als108_STC_sigma = p.as_int();
    }    
  }
}

int LaserImagePos::alg108_getcallbackParameter(const rclcpp::Parameter &p)
{
    if (p.get_name() == "als108_exposure_time") {
        auto k = p.as_int();
        pm.als108_exposure_time=k;
            if(pm.task_num==108){
                _param_camera->set_parameters({rclcpp::Parameter("exposure_time", pm.als108_exposure_time)});}
            return 1;}
    else if(p.get_name() == "als108_center_x") {
        auto k = p.as_int();
        pm.als108_center_x=p.as_int();
            return 1;}
    else if(p.get_name() == "als108_center_y") {
        auto k = p.as_int();
        pm.als108_center_y=p.as_int();
            return 1;}
    else if(p.get_name() == "als108_search_w") {
        auto k = p.as_int();
        pm.als108_search_w=p.as_int();
            return 1;}
    else if(p.get_name() == "als108_search_h") {
        auto k = p.as_int();
        pm.als108_search_h=p.as_int();
            return 1;}
    else if(p.get_name() == "als108_STC_alpha") {
        auto k = p.as_int();
        pm.als108_STC_alpha=p.as_int();
            return 1;}
    else if(p.get_name() == "als108_STC_beta") {
        auto k = p.as_int();
        pm.als108_STC_beta=p.as_int();
            return 1;}
    else if(p.get_name() == "als108_STC_rho") {
        auto k = p.as_int();
        pm.als108_STC_rho=p.as_int();
            return 1;}
    else if(p.get_name() == "als108_STC_sigma") {
        auto k = p.as_int();
        pm.als108_STC_sigma=p.as_int();
            return 1;}                

    return 0;
}

int LaserImagePos::alg108_runimage( cv::Mat &cvimgIn,
                                    std::vector <cv::Point2f> &pointcloud,
                                    std::vector <Targetpoint> &namepoint,
                                    bool &solderjoints,
                                    int step)    //输出结果点信息
{
    Int32 i,j,t,n;
    Myhalcv2::Mat imageIn,imageGasu,imageGasupain;
    Myhalcv2::L_Point32 stepfindST;//结果线1拟合区域,(下方)
    cv::Point2f cv_point;
    Myhalcv2::L_Point32F faxian;
    Myhalcv2::L_Point32 resultfocal;
    Int32 nWidth=cvimgIn.cols;	//输入图像宽
    Int32 nHeight=cvimgIn.rows;	//输入图像高
    Targetpoint targetpoint;


    /*********************/
    //算法参数
    Int32 center_x=pm.als108_center_x;//559;     //特征点位置
    Int32 center_y=pm.als108_center_y;//379
    Int32 search_w=pm.als108_search_w;       //高度搜索区域
    Int32 search_h=pm.als108_search_h;       //宽度搜索区域
    double STC_alpha=pm.als108_STC_alpha/1000.0;    //尺度因子，推荐2.25
    double STC_beta=pm.als108_STC_beta/1000.0;       //形状因子，推荐1
    double STC_rho=pm.als108_STC_rho/1000.0;      //学习率，推荐0.075
    double STC_sigma=pm.als108_STC_sigma/1000.0;    //高斯权重，推荐0.5

    faxian.x=0;
    faxian.y=0;
    
    if(step==2)
    {
        return 0;
    }
    imageIn=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff_image);
    Myhalcv2::CvMatToMat(cvimgIn,&imageIn,cv8uc1_Imagebuff_image);
    imageGasu=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff5);
    Myhalcv2::Mygausspyramid_2levl(imageIn,&imageGasu);  
    if(step!=0)
    {
        imageGasupain=Myhalcv2::MatCreat(nHeight,nWidth,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff8);
    }
    if(step==3)
    {
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        return 0;
    }
    if(firstsearch==0)
    {
        stepfindST.x=center_x>>2;
        stepfindST.y=center_y>>2;
        firstsearch_stx=MAX((Int32)(imageIn.startx),center_x-search_w);
        firstsearch_edx=MIN((Int32)(imageIn.startx+imageIn.width-1),center_x+search_w);
        firstsearch_sty=MAX((Int32)(imageIn.starty),center_y-search_h);
        firstsearch_edy=MIN((Int32)(imageIn.starty+imageIn.height-1),center_y+search_h);
        firstsearch_stx=firstsearch_stx>>2;
        firstsearch_edx=firstsearch_edx>>2;
        firstsearch_sty=firstsearch_sty>>2;
        firstsearch_edy=firstsearch_edy>>2;
        if(step==4)
        {
            Myhalcv2::MatClone(imageGasu,&imageGasupain);
            Myhalcv2::Myrectangle(&imageGasupain,firstsearch_stx,firstsearch_edx,firstsearch_sty,firstsearch_edy,255,1);
            Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
            return 0;
        }
        Myhalcv2::MySTC_init(imageGasu,stepfindST,firstsearch_stx,firstsearch_edx,firstsearch_sty,firstsearch_edy,STC_alpha,STC_beta,STC_rho,STC_sigma);
        firstsearch=1;
        if(step==5)
        {
            Myhalcv2::MatClone(imageGasu,&imageGasupain);
            Myhalcv2::Myrectangle(&imageGasupain,firstsearch_stx,firstsearch_edx,firstsearch_sty,firstsearch_edy,255,1);
            Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
            return 0;
        }
    }
    else
    {
        if(step==4)
        {
            Myhalcv2::MatClone(imageGasu,&imageGasupain);
            Myhalcv2::Myrectangle(&imageGasupain,firstsearch_stx,firstsearch_edx,firstsearch_sty,firstsearch_edy,255,1);
            Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
            return 0;
        }
        Myhalcv2::MySTC_tracker(imageGasu,&stepfindST,&firstsearch_stx,&firstsearch_edx,&firstsearch_sty,&firstsearch_edy);
        if(step==5)
        {
            Myhalcv2::MatClone(imageGasu,&imageGasupain);
            Myhalcv2::Myrectangle(&imageGasupain,firstsearch_stx,firstsearch_edx,firstsearch_sty,firstsearch_edy,255,1);
            Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
            return 0;
        }
    }
    if(step==1)
    {
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        if(cvimgIn.type()==CV_8UC1)
            cv::cvtColor(cvimgIn,cvimgIn,cv::COLOR_GRAY2BGR);
        cv::Rect box;
        box.x=firstsearch_stx;
        box.y=firstsearch_sty;
        box.width=firstsearch_edx-firstsearch_stx+1;
        box.height=firstsearch_edy-firstsearch_sty+1;
        cv::rectangle(cvimgIn,box,cv::Scalar(0,0,255),1);
        cv_point.x=stepfindST.x;
        cv_point.y=stepfindST.y;
        cv::circle(cvimgIn,cv_point,4,cv::Scalar(255,0,255),1);
    }
    resultfocal.x=stepfindST.x*4;
    resultfocal.y=stepfindST.y*4;
   
    cv_point.x=resultfocal.x;
    cv_point.y=resultfocal.y;
    targetpoint.pointf=cv_point;
    targetpoint.name="point_0";
    namepoint.push_back(targetpoint); 
    cv_point.x=faxian.x*1000+resultfocal.x;
    cv_point.y=faxian.y*1000+resultfocal.y;
    targetpoint.pointf=cv_point;
    targetpoint.name="normal";
    namepoint.push_back(targetpoint); 
    cv_point.x=resultfocal.x;
    cv_point.y=resultfocal.y;
    targetpoint.pointf=cv_point;
    targetpoint.name="point_1";
    namepoint.push_back(targetpoint);  
    cv_point.x=resultfocal.x;
    cv_point.y=resultfocal.y;
    targetpoint.pointf=cv_point;
    targetpoint.name="point_2";
    namepoint.push_back(targetpoint);  
    pointcloud.push_back(cv_point);
    pointcloud.push_back(cv_point);
    pointcloud.push_back(cv_point);
    solderjoints=false;
    return 0;
}

}