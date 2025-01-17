#include <cstdio>
#include <modbus/modbus.hpp>
#include <modbus.h>
#include <unistd.h>
#include <fileout/FileOut.h>
#include <fileout/E2proomData.h>
#include <memory>
#include <set>

namespace modbus
{

int Modbus::als102_task_parameter(int ddr,u_int16_t num)
{
    switch(ddr)
    {
        case ALS102_EXPOSURE_TIME_REG_ADD:
            if((int)num>=(int)e2proomdata.als102_exposure_time_min&&(int)num<=(int)e2proomdata.als102_exposure_time_max)
            {
                e2proomdata.als102_exposure_time=num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_exposure_time", num)});
                return 1;
            }
        break;
        case ALS102_PINGJUN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_pingjun_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_pingjun_max)
            {
                e2proomdata.als102_pingjun=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_pingjun", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_B_YANMOFUZHU_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_b_yanmofuzhu_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_b_yanmofuzhu_max)
            {
                e2proomdata.als102_b_yanmofuzhu=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_b_yanmofuzhu", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_B_GUDINGQUYU_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_b_gudingquyu_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_b_gudingquyu_max)
            {
                e2proomdata.als102_b_gudingquyu=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_b_gudingquyu", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_WIDTHLIANTONGDIS_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_widthliantongdis_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_widthliantongdis_max)
            {
                e2proomdata.als102_widthliantongdis=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_widthliantongdis", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_HIGHLIANTONGDIS_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_highliantongdis_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_highliantongdis_max)
            {
                e2proomdata.als102_highliantongdis=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_highliantongdis", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_GUJIAERZHI_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_gujiaerzhi_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_gujiaerzhi_max)
            {
                e2proomdata.als102_gujiaerzhi=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_gujiaerzhi", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_JIGUANGHIGHT_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_jiguanghight_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_jiguanghight_max)
            {
                e2proomdata.als102_jiguanghight=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_jiguanghight", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_JIGUANGLONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_jiguanglong_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_jiguanglong_max)
            {
                e2proomdata.als102_jiguanglong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_jiguanglong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_JIGUANGKUANDU_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_jiguangkuandu_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_jiguangkuandu_max)
            {
                e2proomdata.als102_jiguangkuandu=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_jiguangkuandu", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_UPDIF_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Updif_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Updif_max)
            {
                e2proomdata.als102_Updif=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Updif", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_UPDIFMIN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Updifmin_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Updifmin_max)
            {
                e2proomdata.als102_Updifmin=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Updifmin", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_UPLONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Uplong_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Uplong_max)
            {
                e2proomdata.als102_Uplong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Uplong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DOWNDIF_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Downdif_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Downdif_max)
            {
                e2proomdata.als102_Downdif=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Downdif", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DOWNDIFMIN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Downdifmin_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Downdifmin_max)
            {
                e2proomdata.als102_Downdifmin=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Downdifmin", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DOWNDLONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Downdlong_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Downdlong_max)
            {
                e2proomdata.als102_Downdlong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Downdlong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_ST_DOWN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_St_Down_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_St_Down_max)
            {
                e2proomdata.als102_St_Down=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_St_Down", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_ED_DOWN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Ed_Down_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Ed_Down_max)
            {
                e2proomdata.als102_Ed_Down=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Ed_Down", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_ST_UP_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_St_Up_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_St_Up_max)
            {
                e2proomdata.als102_St_Up=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_St_Up", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_ED_UP_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Ed_Up_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Ed_Up_max)
            {
                e2proomdata.als102_Ed_Up=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Ed_Up", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_UPDIF2_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Updif2_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Updif2_max)
            {
                e2proomdata.als102_Updif2=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Updif2", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_UPDIFMIN2_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Updifmin2_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Updifmin2_max)
            {
                e2proomdata.als102_Updifmin2=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Updifmin2", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DIS_CENTER_ST_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_dis_center_st_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_dis_center_st_max)
            {
                e2proomdata.als102_dis_center_st=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_dis_center_st", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DIS_CENTER_ED_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_dis_center_ed_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_dis_center_ed_max)
            {
                e2proomdata.als102_dis_center_ed=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_dis_center_ed", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_B_OPENGUDINGDIMIAN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_b_opengudingdimian_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_b_opengudingdimian_max)
            {
                e2proomdata.als102_b_opengudingdimian=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_b_opengudingdimian", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DIMIANPANGDINGJULI_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_dimianpangdingjuli_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_dimianpangdingjuli_max)
            {
                e2proomdata.als102_dimianpangdingjuli=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_dimianpangdingjuli", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DIMIANPINGJUNSHUNUM_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_dimianpingjunshunum_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_dimianpingjunshunum_max)
            {
                e2proomdata.als102_dimianpingjunshunum=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_dimianpingjunshunum", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DIS_CENTER_ST2_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_dis_center_st2_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_dis_center_st2_max)
            {
                e2proomdata.als102_dis_center_st2=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_dis_center_st2", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DIS_CENTER_ED2_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_dis_center_ed2_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_dis_center_ed2_max)
            {
                e2proomdata.als102_dis_center_ed2=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_dis_center_ed2", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DIS_CENTER_ST3_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_dis_center_st3_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_dis_center_st3_max)
            {
                e2proomdata.als102_dis_center_st3=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_dis_center_st3", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_DIS_CENTER_ED3_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_dis_center_ed3_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_dis_center_ed3_max)
            {
                e2proomdata.als102_dis_center_ed3=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_dis_center_ed3", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_XUEXIJULI_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_xuexijuli_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_xuexijuli_max)
            {
                e2proomdata.als102_xuexijuli=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_xuexijuli", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_B_PINGPOWENGDING_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_b_pingpowending_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_b_pingpowending_max)
            {
                e2proomdata.als102_b_pingpowending=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_b_pingpowending", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_PINGPOWENGDING_DIS_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_pingpowending_dis_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_pingpowending_dis_max)
            {
                e2proomdata.als102_pingpowending_dis=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_pingpowending_dis", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_B_XIELVOPEN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_b_xielvopen_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_b_xielvopen_max)
            {
                e2proomdata.als102_b_xielvopen=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_b_xielvopen", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_XIELVFANWEI_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_xielvfanwei_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_xielvfanwei_max)
            {
                e2proomdata.als102_xielvfanwei=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_xielvfanwei", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_UPLONG2_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_Uplong2_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_Uplong2_max)
            {
                e2proomdata.als102_Uplong2=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_Uplong2", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_CEBANKONGDONGDIS_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_cebankongdongdis_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_cebankongdongdis_max)
            {
                e2proomdata.als102_cebankongdongdis=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_cebankongdongdis", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_QIATOUQUWEI_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_qiatouquweijuli_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_qiatouquweijuli_max)
            {
                e2proomdata.als102_qiatouquweijuli=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_qiatouquweijuli", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_ANSWERPOINT_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_answerpoint_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_answerpoint_max)
            {
                e2proomdata.als102_answerpoint=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_answerpoint", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_B_KALMANFILTER_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_b_KalmanFilter_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_b_KalmanFilter_max)
            {
                e2proomdata.als102_b_KalmanFilter=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_b_KalmanFilter", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_KALMANQF_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_KalmanQF_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_KalmanQF_max)
            {
                e2proomdata.als102_KalmanQF=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_KalmanQF", (int16_t)num)});
                return 1;
            }
        break;
        case ALS102_KALMANRF_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als102_KalmanRF_min&&(int)((int16_t)num)<=(int)e2proomdata.als102_KalmanRF_max)
            {
                e2proomdata.als102_KalmanRF=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als102_KalmanRF", (int16_t)num)});
                return 1;
            }
        break;

        case ALS102_INIT_REG_ADD:
            if(num==1)
            {
                e2proomdata.init_als102_para();
                init_als102_parameter();
                parameterport_mapping->tab_registers[ALS102_INIT_REG_ADD]=0;
            }
            return 1;
  
        default:
        break;
    }
    return 0;
}

void Modbus::init_als102_parameter()
{
    parameterport_mapping->tab_registers[ALS102_EXPOSURE_TIME_REG_ADD]=e2proomdata.als102_exposure_time;
    parameterport_mapping->tab_registers[ALS102_PINGJUN_REG_ADD]=e2proomdata.als102_pingjun;
    parameterport_mapping->tab_registers[ALS102_B_YANMOFUZHU_REG_ADD]=e2proomdata.als102_b_yanmofuzhu;
    parameterport_mapping->tab_registers[ALS102_B_GUDINGQUYU_REG_ADD]=e2proomdata.als102_b_gudingquyu;
    parameterport_mapping->tab_registers[ALS102_WIDTHLIANTONGDIS_REG_ADD]=e2proomdata.als102_widthliantongdis;
    parameterport_mapping->tab_registers[ALS102_HIGHLIANTONGDIS_REG_ADD]=e2proomdata.als102_highliantongdis;
    parameterport_mapping->tab_registers[ALS102_GUJIAERZHI_REG_ADD]=e2proomdata.als102_gujiaerzhi;
    parameterport_mapping->tab_registers[ALS102_JIGUANGHIGHT_REG_ADD]=e2proomdata.als102_jiguanghight;
    parameterport_mapping->tab_registers[ALS102_JIGUANGLONG_REG_ADD]=e2proomdata.als102_jiguanglong;
    parameterport_mapping->tab_registers[ALS102_JIGUANGKUANDU_REG_ADD]=e2proomdata.als102_jiguangkuandu;
    parameterport_mapping->tab_registers[ALS102_UPDIF_REG_ADD]=e2proomdata.als102_Updif;
    parameterport_mapping->tab_registers[ALS102_UPDIFMIN_REG_ADD]=e2proomdata.als102_Updifmin;
    parameterport_mapping->tab_registers[ALS102_UPLONG_REG_ADD]=e2proomdata.als102_Uplong;
    parameterport_mapping->tab_registers[ALS102_DOWNDIF_REG_ADD]=e2proomdata.als102_Downdif;
    parameterport_mapping->tab_registers[ALS102_DOWNDIFMIN_REG_ADD]=e2proomdata.als102_Downdifmin;
    parameterport_mapping->tab_registers[ALS102_DOWNDLONG_REG_ADD]=e2proomdata.als102_Downdlong;
    parameterport_mapping->tab_registers[ALS102_ST_DOWN_REG_ADD]=e2proomdata.als102_St_Down;
    parameterport_mapping->tab_registers[ALS102_ED_DOWN_REG_ADD]=e2proomdata.als102_Ed_Down;
    parameterport_mapping->tab_registers[ALS102_ST_UP_REG_ADD]=e2proomdata.als102_St_Up;
    parameterport_mapping->tab_registers[ALS102_ED_UP_REG_ADD]=e2proomdata.als102_Ed_Up;
    parameterport_mapping->tab_registers[ALS102_UPDIF2_REG_ADD]=e2proomdata.als102_Updif2;
    parameterport_mapping->tab_registers[ALS102_UPDIFMIN2_REG_ADD]=e2proomdata.als102_Updifmin2;
    parameterport_mapping->tab_registers[ALS102_DIS_CENTER_ST_REG_ADD]=e2proomdata.als102_dis_center_st;
    parameterport_mapping->tab_registers[ALS102_DIS_CENTER_ED_REG_ADD]=e2proomdata.als102_dis_center_ed;
    parameterport_mapping->tab_registers[ALS102_B_OPENGUDINGDIMIAN_REG_ADD]=e2proomdata.als102_b_opengudingdimian;
    parameterport_mapping->tab_registers[ALS102_DIMIANPANGDINGJULI_REG_ADD]=e2proomdata.als102_dimianpangdingjuli;
    parameterport_mapping->tab_registers[ALS102_DIMIANPINGJUNSHUNUM_REG_ADD]=e2proomdata.als102_dimianpingjunshunum;
    parameterport_mapping->tab_registers[ALS102_DIS_CENTER_ST2_REG_ADD]=e2proomdata.als102_dis_center_st2;
    parameterport_mapping->tab_registers[ALS102_DIS_CENTER_ED2_REG_ADD]=e2proomdata.als102_dis_center_ed2;
    parameterport_mapping->tab_registers[ALS102_DIS_CENTER_ST3_REG_ADD]=e2proomdata.als102_dis_center_st3;
    parameterport_mapping->tab_registers[ALS102_DIS_CENTER_ED3_REG_ADD]=e2proomdata.als102_dis_center_ed3;
    parameterport_mapping->tab_registers[ALS102_XUEXIJULI_REG_ADD]=e2proomdata.als102_xuexijuli;
    parameterport_mapping->tab_registers[ALS102_B_PINGPOWENGDING_REG_ADD]=e2proomdata.als102_b_pingpowending;
    parameterport_mapping->tab_registers[ALS102_PINGPOWENGDING_DIS_REG_ADD]=e2proomdata.als102_pingpowending_dis;
    parameterport_mapping->tab_registers[ALS102_B_XIELVOPEN_REG_ADD]=e2proomdata.als102_b_xielvopen;
    parameterport_mapping->tab_registers[ALS102_XIELVFANWEI_REG_ADD]=e2proomdata.als102_xielvfanwei;
    parameterport_mapping->tab_registers[ALS102_UPLONG2_REG_ADD]=e2proomdata.als102_Uplong2;
    parameterport_mapping->tab_registers[ALS102_CEBANKONGDONGDIS_REG_ADD]=e2proomdata.als102_cebankongdongdis;
    parameterport_mapping->tab_registers[ALS102_QIATOUQUWEI_REG_ADD]=e2proomdata.als102_qiatouquweijuli;
    parameterport_mapping->tab_registers[ALS102_ANSWERPOINT_REG_ADD]=e2proomdata.als102_answerpoint;
    parameterport_mapping->tab_registers[ALS102_B_KALMANFILTER_REG_ADD]=e2proomdata.als102_b_KalmanFilter;   
    parameterport_mapping->tab_registers[ALS102_KALMANQF_REG_ADD]=e2proomdata.als102_KalmanQF;  
    parameterport_mapping->tab_registers[ALS102_KALMANRF_REG_ADD]=e2proomdata.als102_KalmanRF;  
}

}