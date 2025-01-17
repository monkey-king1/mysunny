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

int Modbus::als106_task_parameter(int ddr,u_int16_t num)
{
    switch(ddr)
    {
        case ALS106_EXPOSURE_TIME_REG_ADD:
            if((int)num>=(int)e2proomdata.als106_exposure_time_min&&(int)num<=(int)e2proomdata.als106_exposure_time_max)
            {
                e2proomdata.als106_exposure_time=num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_exposure_time", num)});
                return 1;
            }
        break;
        case ALS106_PINGJUN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pingjun_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pingjun_max)
            {
                e2proomdata.als106_pingjun=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pingjun", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_B_YANMOFUZHU_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_b_yanmofuzhu_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_b_yanmofuzhu_max)
            {
                e2proomdata.als106_b_yanmofuzhu=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_b_yanmofuzhu", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_B_GUDINGQUYU_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_b_gudingquyu_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_b_gudingquyu_max)
            {
                e2proomdata.als106_b_gudingquyu=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_b_gudingquyu", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_WIDTHLIANTONGDIS_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_widthliantongdis_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_widthliantongdis_max)
            {
                e2proomdata.als106_widthliantongdis=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_widthliantongdis", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_HIGHLIANTONGDIS_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_highliantongdis_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_highliantongdis_max)
            {
                e2proomdata.als106_highliantongdis=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_highliantongdis", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_GUJIAERZHI_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_gujiaerzhi_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_gujiaerzhi_max)
            {
                e2proomdata.als106_gujiaerzhi=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_gujiaerzhi", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_JIGUANGHIGHT_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_jiguanghight_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_jiguanghight_max)
            {   
                e2proomdata.als106_jiguanghight=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_jiguanghight", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_JIGUANGLONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_jiguanglong_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_jiguanglong_max)
            {
                e2proomdata.als106_jiguanglong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_jiguanglong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_JIGUANGKUANDU_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_jiguangkuandu_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_jiguangkuandu_max)
            {
                e2proomdata.als106_jiguangkuandu=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_jiguangkuandu", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_B_CUT_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_b_cut_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_b_cut_max)
            {
                e2proomdata.als106_b_cut=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_b_cut", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_CUTLEFT_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_cutleft_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_cutleft_max)
            {
                e2proomdata.als106_cutleft=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_cutleft", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_CUTRIGHT_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_cutright_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_cutright_max)
            {
                e2proomdata.als106_cutright=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_cutright", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_CUTTOP_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_cuttop_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_cuttop_max)
            {
                e2proomdata.als106_cuttop=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_cuttop", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_CUTDEEP_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_cutdeep_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_cutdeep_max)
            {
                e2proomdata.als106_cutdeep=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_cutdeep", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_DIFMIN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_difmin_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_difmin_max)
            {
                e2proomdata.als106_difmin=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_difmin", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_SIDELONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Sidelong_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Sidelong_max)
            {
                e2proomdata.als106_Sidelong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Sidelong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_UPBACK_ST_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Upback_st_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Upback_st_max)
            {
                e2proomdata.als106_Upback_st=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Upback_st", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_UPBACK_ED_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Upback_ed_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Upback_ed_max)
            {
                e2proomdata.als106_Upback_ed=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Upback_ed", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_DOWNBACK_ST_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Downback_st_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Downback_st_max)
            {
                e2proomdata.als106_Downback_st=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Downback_st", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_DOWNBACK_ED_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Downback_ed_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Downback_ed_max)
            {
                e2proomdata.als106_Downback_ed=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Downback_ed", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_UPMINDIS_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Upmindis_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Upmindis_max)
            {
                e2proomdata.als106_Upmindis=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Upmindis", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_DOWNMINDIS_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Downmindis_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Downmindis_max)
            {
                e2proomdata.als106_Downmindis=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Downmindis", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_UPLONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Uplong_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Uplong_max)
            {
                e2proomdata.als106_Uplong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Uplong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_DOWNDLONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_Downdlong_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_Downdlong_max)
            {
                e2proomdata.als106_Downdlong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_Downdlong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_DISCENTER_ED_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_dis_center_ed_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_dis_center_ed_max)
            {
                e2proomdata.als106_dis_center_ed=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_dis_center_ed", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_DUANDIANJULI_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_duandianjuli_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_duandianjuli_max)
            {
                e2proomdata.als106_duandianjuli=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_duandianjuli", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUMOD_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokoumod_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokoumod_max)
            {
                e2proomdata.als106_pokoumod=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokoumod", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUUPDIF_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouUpdif_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouUpdif_max)
            {
                e2proomdata.als106_pokouUpdif=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouUpdif", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUUPDIFMIN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouUpdifmin_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouUpdifmin_max)
            {
                e2proomdata.als106_pokouUpdifmin=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouUpdifmin", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUUPBACK_ST_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouUpback_st_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouUpback_st_max)
            {
                e2proomdata.als106_pokouUpback_st=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouUpback_st", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUUPBACK_ED_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouUpback_ed_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouUpback_ed_max)
            {
                e2proomdata.als106_pokouUpback_ed=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouUpback_ed", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUDOWNDIF_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouDowndif_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouDowndif_max)
            {
                e2proomdata.als106_pokouDowndif=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouDowndif", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUDOWNDIFMIN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouDowndifmin_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouDowndifmin_max)
            {
                e2proomdata.als106_pokouDowndifmin=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouDowndifmin", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUDOWNBACK_ST_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouDownback_st_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouDownback_st_max)
            {
                e2proomdata.als106_pokouDownback_st=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouDownback_st", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUDOWNBACK_ED_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouDownback_ed_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouDownback_ed_max)
            {
                e2proomdata.als106_pokouDownback_ed=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouDownback_ed", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUUPLONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouUplong_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouUplong_max)
            {
                e2proomdata.als106_pokouUplong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouUplong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUDOWNLONG_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouDownlong_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouDownlong_max)
            {
                e2proomdata.als106_pokouDownlong=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouDownlong", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_B_POKOUYAOBIAN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_b_pokouyaobian_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_b_pokouyaobian_max)
            {
                e2proomdata.als106_b_pokouyaobian=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_b_pokouyaobian", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUYAOBIANHSIZE_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouyaobianHsize_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouyaobianHsize_max)
            {
                e2proomdata.als106_pokouyaobianHsize=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouyaobianHsize", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUDUANXIANERZHI_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokouduanxianerzhi_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokouduanxianerzhi_max)
            {
                e2proomdata.als106_pokouduanxianerzhi=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokouduanxianerzhi", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUSEARCHDECTANCEMAX_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokousearchdectancemax_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokousearchdectancemax_max)
            {
                e2proomdata.als106_pokousearchdectancemax=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokousearchdectancemax", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_POKOUSEARCHDECTANCEMIN_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_pokousearchdectancemin_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_pokousearchdectancemin_max)
            {
                e2proomdata.als106_pokousearchdectancemin=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_pokousearchdectancemin", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_ANSWERPOINT_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_answerpoint_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_answerpoint_max)
            {
                e2proomdata.als106_answerpoint=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_answerpoint", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_B_KALMANFILTER_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_b_KalmanFilter_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_b_KalmanFilter_max)
            {
                e2proomdata.als106_b_KalmanFilter=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_b_KalmanFilter", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_KALMANQF_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_KalmanQF_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_KalmanQF_max)
            {
                e2proomdata.als106_KalmanQF=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_KalmanQF", (int16_t)num)});
                return 1;
            }
        break;
        case ALS106_KALMANRF_REG_ADD:
            if((int)((int16_t)num)>=(int)e2proomdata.als106_KalmanRF_min&&(int)((int16_t)num)<=(int)e2proomdata.als106_KalmanRF_max)
            {
                e2proomdata.als106_KalmanRF=(int16_t)num;
                _param_laserimagepos->set_parameters({rclcpp::Parameter("als106_KalmanRF", (int16_t)num)});
                return 1;
            }
        break;

        case ALS106_INIT_REG_ADD:
            if(num==1)
            {
                e2proomdata.init_als106_para();
                init_als106_parameter();
                parameterport_mapping->tab_registers[ALS106_INIT_REG_ADD]=0;
            }
            return 1;
  
        default:
        break;
    }
    return 0;
}

void Modbus::init_als106_parameter()
{
    parameterport_mapping->tab_registers[ALS106_EXPOSURE_TIME_REG_ADD]=e2proomdata.als106_exposure_time;
    parameterport_mapping->tab_registers[ALS106_PINGJUN_REG_ADD]=e2proomdata.als106_pingjun;
    parameterport_mapping->tab_registers[ALS106_B_YANMOFUZHU_REG_ADD]=e2proomdata.als106_b_yanmofuzhu;
    parameterport_mapping->tab_registers[ALS102_B_GUDINGQUYU_REG_ADD]=e2proomdata.als102_b_gudingquyu;
    parameterport_mapping->tab_registers[ALS106_WIDTHLIANTONGDIS_REG_ADD]=e2proomdata.als106_widthliantongdis;
    parameterport_mapping->tab_registers[ALS106_HIGHLIANTONGDIS_REG_ADD]=e2proomdata.als106_highliantongdis;
    parameterport_mapping->tab_registers[ALS106_GUJIAERZHI_REG_ADD]=e2proomdata.als106_gujiaerzhi;
    parameterport_mapping->tab_registers[ALS106_JIGUANGHIGHT_REG_ADD]=e2proomdata.als106_jiguanghight;
    parameterport_mapping->tab_registers[ALS106_JIGUANGLONG_REG_ADD]=e2proomdata.als106_jiguanglong;
    parameterport_mapping->tab_registers[ALS106_JIGUANGKUANDU_REG_ADD]=e2proomdata.als106_jiguangkuandu;
    parameterport_mapping->tab_registers[ALS106_B_CUT_REG_ADD]=e2proomdata.als106_b_cut;
    parameterport_mapping->tab_registers[ALS106_CUTLEFT_REG_ADD]=e2proomdata.als106_cutleft;
    parameterport_mapping->tab_registers[ALS106_CUTRIGHT_REG_ADD]=e2proomdata.als106_cutright;
    parameterport_mapping->tab_registers[ALS106_CUTTOP_REG_ADD]=e2proomdata.als106_cuttop;
    parameterport_mapping->tab_registers[ALS106_CUTDEEP_REG_ADD]=e2proomdata.als106_cutdeep;
    parameterport_mapping->tab_registers[ALS106_DIFMIN_REG_ADD]=e2proomdata.als106_difmin;
    parameterport_mapping->tab_registers[ALS106_SIDELONG_REG_ADD]=e2proomdata.als106_Sidelong;
    parameterport_mapping->tab_registers[ALS106_UPBACK_ST_REG_ADD]=e2proomdata.als106_Upback_st;
    parameterport_mapping->tab_registers[ALS106_UPBACK_ED_REG_ADD]=e2proomdata.als106_Upback_ed;
    parameterport_mapping->tab_registers[ALS106_DOWNBACK_ST_REG_ADD]=e2proomdata.als106_Downback_st;
    parameterport_mapping->tab_registers[ALS106_DOWNBACK_ED_REG_ADD]=e2proomdata.als106_Downback_ed;
    parameterport_mapping->tab_registers[ALS106_UPMINDIS_REG_ADD]=e2proomdata.als106_Upmindis;
    parameterport_mapping->tab_registers[ALS106_DOWNMINDIS_REG_ADD]=e2proomdata.als106_Downmindis;
    parameterport_mapping->tab_registers[ALS106_UPLONG_REG_ADD]=e2proomdata.als106_Uplong;
    parameterport_mapping->tab_registers[ALS106_DOWNDLONG_REG_ADD]=e2proomdata.als106_Downdlong;
    parameterport_mapping->tab_registers[ALS106_DISCENTER_ED_REG_ADD]=e2proomdata.als106_dis_center_ed;
    parameterport_mapping->tab_registers[ALS106_DUANDIANJULI_REG_ADD]=e2proomdata.als106_duandianjuli;
    parameterport_mapping->tab_registers[ALS106_POKOUMOD_REG_ADD]=e2proomdata.als106_pokoumod;
    parameterport_mapping->tab_registers[ALS106_POKOUUPDIF_REG_ADD]=e2proomdata.als106_pokouUpdif;
    parameterport_mapping->tab_registers[ALS106_POKOUUPDIFMIN_REG_ADD]=e2proomdata.als106_pokouUpdifmin;
    parameterport_mapping->tab_registers[ALS106_POKOUUPBACK_ST_REG_ADD]=e2proomdata.als106_pokouUpback_st;
    parameterport_mapping->tab_registers[ALS106_POKOUUPBACK_ED_REG_ADD]=e2proomdata.als106_pokouUpback_ed;
    parameterport_mapping->tab_registers[ALS106_POKOUDOWNDIF_REG_ADD]=e2proomdata.als106_pokouDowndif;
    parameterport_mapping->tab_registers[ALS106_POKOUDOWNDIFMIN_REG_ADD]=e2proomdata.als106_pokouDowndifmin;
    parameterport_mapping->tab_registers[ALS106_POKOUDOWNBACK_ST_REG_ADD]=e2proomdata.als106_pokouDownback_st;
    parameterport_mapping->tab_registers[ALS106_POKOUDOWNBACK_ED_REG_ADD]=e2proomdata.als106_pokouDownback_ed;
    parameterport_mapping->tab_registers[ALS106_POKOUUPLONG_REG_ADD]=e2proomdata.als106_pokouUplong;
    parameterport_mapping->tab_registers[ALS106_POKOUDOWNLONG_REG_ADD]=e2proomdata.als106_pokouDownlong;
    parameterport_mapping->tab_registers[ALS106_B_POKOUYAOBIAN_REG_ADD]=e2proomdata.als106_b_pokouyaobian;
    parameterport_mapping->tab_registers[ALS106_POKOUYAOBIANHSIZE_REG_ADD]=e2proomdata.als106_pokouyaobianHsize;
    parameterport_mapping->tab_registers[ALS106_POKOUDUANXIANERZHI_REG_ADD]=e2proomdata.als106_pokouduanxianerzhi;
    parameterport_mapping->tab_registers[ALS106_POKOUSEARCHDECTANCEMAX_REG_ADD]=e2proomdata.als106_pokousearchdectancemax;
    parameterport_mapping->tab_registers[ALS106_POKOUSEARCHDECTANCEMIN_REG_ADD]=e2proomdata.als106_pokousearchdectancemin;
    parameterport_mapping->tab_registers[ALS106_ANSWERPOINT_REG_ADD]=e2proomdata.als106_answerpoint;
    parameterport_mapping->tab_registers[ALS106_B_KALMANFILTER_REG_ADD]=e2proomdata.als106_b_KalmanFilter;   
    parameterport_mapping->tab_registers[ALS106_KALMANQF_REG_ADD]=e2proomdata.als106_KalmanQF;  
    parameterport_mapping->tab_registers[ALS106_KALMANRF_REG_ADD]=e2proomdata.als106_KalmanRF;  
}

}