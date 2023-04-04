#ifndef MY_PCLOUT_HPP_
#define MY_PCLOUT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "fileout/calibration.h"
#include "fileout/MyPlcFunction.h"
#include "tutorial_interfaces/msg/if_algorhmitcloud.hpp"
#include "tutorial_interfaces/msg/if_algorhmitrobpos.hpp"
#include "tutorial_interfaces/msg/if_algorhmitrobcloud.hpp"
#include "unistd.h"

namespace my_pclout
{
using rcl_interfaces::msg::SetParametersResult;
using std::placeholders::_1;
using tutorial_interfaces::msg::IfAlgorhmitcloud;
using tutorial_interfaces::msg::IfAlgorhmitrobpos;
using tutorial_interfaces::msg::IfAlgorhmitroblinecloud;
using tutorial_interfaces::msg::IfAlgorhmitrobcloud;

const std::vector<std::string> KEYS = {"pData_En",
                                       "pData_demdlg_R",
                                       "pData_demdlg_T",
                                       "pData_matrix_camera2plane",
                                       "pData_matrix_plane2robot",
                                       "PData_cal_posture",
                                       "PData_eye_hand_calibrationmode",
                                       "b_pclpush",        //开始导入点集
                                       "save_pcl"          //保存点集
                                       "clear_pcl"};       //清空点集  

class My_Pclout : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Laser Line Center object.
     *
     * Initialize publisher.
     * Create an inner implementation.
     * Initialize subscription.
     * Print success if all done.
     * @param options Encapsulation of options for node initialization.
     */
    explicit My_Pclout(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Destroy the Laser Line Center object.
     *
     * Release subscription.
     * Wake up all workers.
     * Wake up the manager.
     * Synchronize with all threads, wait for its return.
     * Release publisher.
     * Print success if all done.
     * Throw no exception.
     */
    virtual ~My_Pclout();

    OnSetParametersCallbackHandle::SharedPtr _handle;

    Int8 pData_En;
    Eigen::Matrix3d pData_demdlg_R;
    Eigen::Vector3d pData_demdlg_T;
    cv::Mat pData_matrix_camera2plane;
    cv::Mat pData_matrix_plane2robot;
    CAL_POSTURE PData_cal_posture; //P变量姿态内外旋
    Eye_Hand_calibrationmode PData_eye_hand_calibrationmode;//P寄存器激光器安装方式

    RobPos rob;
    leaser_pos leaserpos;//激光器坐标

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pcl_lineclould;       //轮廓检测结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pcl_deepclould;       //深度检测结果点云

    bool b_pclpush;//点云存入开关

private:

    void _declare_parameters();

    const char * _sub_cloudresult_name = "~/cloudresult";

    const char * _sub_robposresult_name = "~/input_robpos";

    const char * _pub_pcllineresult_name = "~/pcllineresult";

    std::thread _cloudresulttcpthread;

    rclcpp::Subscription<tutorial_interfaces::msg::IfAlgorhmitcloud>::SharedPtr subscription_cloud_result;

    rclcpp::Subscription<tutorial_interfaces::msg::IfAlgorhmitrobpos>::SharedPtr subcription_pos_result;

    rclcpp::Publisher<IfAlgorhmitroblinecloud>::SharedPtr _pub_robline;

    void cloud_result_callback(const tutorial_interfaces::msg::IfAlgorhmitcloud msg);

    void robpos_result_callback(const tutorial_interfaces::msg::IfAlgorhmitrobpos msg);

    void _cloudresult();  
    
};

}

#endif