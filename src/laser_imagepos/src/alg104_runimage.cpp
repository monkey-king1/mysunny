#include "laser_imagepos/laser_imagepos.hpp"

#include "opencv2/opencv.hpp"
namespace laser_imagepos
{

using rcl_interfaces::msg::SetParametersResult;

void LaserImagePos::alg104_declare_parameters()
{
    this->declare_parameter("als104_exposure_time", pm.als104_exposure_time);
    this->declare_parameter("als104_pingjun", pm.als104_pingjun);
    this->declare_parameter("als104_b_yanmofuzhu", pm.als104_b_yanmofuzhu);
    this->declare_parameter("als104_b_gudingquyu", pm.als104_b_gudingquyu);
    this->declare_parameter("als104_widthliantongdis", pm.als104_widthliantongdis);
    this->declare_parameter("als104_highliantongdis", pm.als104_highliantongdis);
    this->declare_parameter("als104_gujiaerzhi", pm.als104_gujiaerzhi);
    this->declare_parameter("als104_jiguanghight", pm.als104_jiguanghight);
    this->declare_parameter("als104_jiguanglong", pm.als104_jiguanglong);
    this->declare_parameter("als104_jiguangkuandu", pm.als104_jiguangkuandu);
    this->declare_parameter("als104_Updif", pm.als104_Updif);
    this->declare_parameter("als104_Updifmin", pm.als104_Updifmin);
    this->declare_parameter("als104_Uplong", pm.als104_Uplong);
    this->declare_parameter("als104_Downdif", pm.als104_Downdif);
    this->declare_parameter("als104_Downdifmin", pm.als104_Downdifmin);
    this->declare_parameter("als104_Downdlong", pm.als104_Downdlong);
    this->declare_parameter("als104_dis_center_st", pm.als104_dis_center_st);
    this->declare_parameter("als104_dis_center_ed", pm.als104_dis_center_ed);
    this->declare_parameter("als104_b_KalmanFilter", pm.als104_b_KalmanFilter); 
    this->declare_parameter("als104_KalmanQF", pm.als104_KalmanQF);
    this->declare_parameter("als104_KalmanRF", pm.als104_KalmanRF);
}

void LaserImagePos::alg104_update_parameters()
{
  const auto & vp = this->get_parameters(KEYS_ALS104);
  for (const auto & p : vp) {
    if (p.get_name() == "als104_exposure_time") {
      pm.als104_exposure_time = p.as_int();
    } else if (p.get_name() == "als104_pingjun") {
      pm.als104_pingjun = p.as_int();
    }
    else if (p.get_name() == "als104_b_yanmofuzhu") {
      pm.als104_b_yanmofuzhu = p.as_int();
    }
    else if (p.get_name() == "als104_b_gudingquyu") {
      pm.als104_b_gudingquyu = p.as_int();
    }
    else if (p.get_name() == "als104_widthliantongdis") {
      pm.als104_widthliantongdis = p.as_int();
    }
    else if (p.get_name() == "als104_highliantongdis") {
      pm.als104_highliantongdis = p.as_int();
    }
    else if (p.get_name() == "als104_gujiaerzhi") {
      pm.als104_gujiaerzhi = p.as_int();
    }
    else if (p.get_name() == "als104_jiguanghight") {
      pm.als104_jiguanghight = p.as_int();
    }
    else if (p.get_name() == "als104_jiguanglong") {
      pm.als104_jiguanglong = p.as_int();
    }
    else if (p.get_name() == "als104_jiguangkuandu") {
      pm.als104_jiguangkuandu = p.as_int();
    }
    else if (p.get_name() == "als104_Updif") {
      pm.als104_Updif = p.as_int();
    }
    else if (p.get_name() == "als104_Updifmin") {
      pm.als104_Updifmin = p.as_int();
    }
    else if (p.get_name() == "als104_Uplong") {
      pm.als104_Uplong = p.as_int();
    }
    else if (p.get_name() == "als104_Downdif") {
      pm.als104_Downdif = p.as_int();
    }
    else if (p.get_name() == "als104_Downdifmin") {
      pm.als104_Downdifmin = p.as_int();
    }
    else if (p.get_name() == "als104_Downdlong") {
      pm.als104_Downdlong = p.as_int();
    }
    else if (p.get_name() == "als104_dis_center_st") {
      pm.als104_dis_center_st = p.as_int();
    }
    else if (p.get_name() == "als104_dis_center_ed") {
      pm.als104_dis_center_ed = p.as_int();
    }
    else if (p.get_name() == "als104_b_KalmanFilter") {
      pm.als104_b_KalmanFilter = p.as_int();
    }
    else if (p.get_name() == "als104_KalmanQF") {
      pm.als104_KalmanQF = p.as_int();
    }
    else if (p.get_name() == "als104_KalmanRF") {
      pm.als104_KalmanRF = p.as_int();
    }
  }
}

int LaserImagePos::alg104_getcallbackParameter(const rclcpp::Parameter &p)
{
    if (p.get_name() == "als104_exposure_time") {
        auto k = p.as_int();
        pm.als104_exposure_time=k;
            if(pm.task_num==104){
                _param_camera->set_parameters({rclcpp::Parameter("exposure_time", pm.als104_exposure_time)});}
            return 1;}
    else if(p.get_name() == "als104_pingjun") {
        auto k = p.as_int();
        pm.als104_pingjun=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_b_yanmofuzhu") {
        auto k = p.as_int();
        pm.als104_b_yanmofuzhu=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_b_gudingquyu") {
        auto k = p.as_int();
        pm.als104_b_gudingquyu=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_widthliantongdis") {
        auto k = p.as_int();
        pm.als104_widthliantongdis=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_highliantongdis") {
        auto k = p.as_int();
        pm.als104_highliantongdis=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_gujiaerzhi") {
        auto k = p.as_int();
        pm.als104_gujiaerzhi=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_jiguanghight") {
        auto k = p.as_int();
        pm.als104_jiguanghight=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_jiguanglong") {
        auto k = p.as_int();
        pm.als104_jiguanglong=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_jiguangkuandu") {
        auto k = p.as_int();
        pm.als104_jiguangkuandu=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_Updif") {
        auto k = p.as_int();
        pm.als104_Updif=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_Updifmin") {
        auto k = p.as_int();
        pm.als104_Updifmin=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_Uplong") {
        auto k = p.as_int();
        pm.als104_Uplong=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_Downdif") {
        auto k = p.as_int();
        pm.als104_Downdif=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_Downdifmin") {
        auto k = p.as_int();
        pm.als104_Downdifmin=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_Downdlong") {
        auto k = p.as_int();
        pm.als104_Downdlong=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_dis_center_st") {
        auto k = p.as_int();
        pm.als104_dis_center_st=p.as_int();
            return 1;}  
    else if(p.get_name() == "als104_dis_center_ed") {
        auto k = p.as_int();
        pm.als104_dis_center_ed=p.as_int();
            return 1;}    
    else if(p.get_name() == "als104_b_KalmanFilter") {
        auto k = p.as_int();
        pm.als104_b_KalmanFilter=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_KalmanQF") {
        auto k = p.as_int();
        pm.als104_KalmanQF=p.as_int();
            return 1;}
    else if(p.get_name() == "als104_KalmanRF") {
        auto k = p.as_int();
        pm.als104_KalmanRF=p.as_int();
            return 1;}  
    return 0;
}

int LaserImagePos::alg104_runimage( cv::Mat &cvimgIn,
                                    std::vector <cv::Point2f> &pointcloud,
                                    std::vector <Targetpoint> &namepoint,
                                    bool &solderjoints,
                                    int step)    //输出结果点信息
{
    Int32 i,j,n;
    Int32 filenum=0,asfilenum=0;
    Myhalcv2::Mat imageIn,imageGasu,imageGasupain,imageBry,m_tempmatIn,m_matMask;
    Myhalcv2::Mat m_brygujia;
    Int32 nWidth=cvimgIn.cols;	//输入图像宽
    Int32 nHeight=cvimgIn.rows;	//输入图像高
    Uint8 bryvalue;
    Int32 i32_bryvalue;

    Int16 filterdata2[40];

    Myhalcv2::L_Point32 f_center={-1,-1};
    Int32 X_Linestarty=0;
    Int32 X_Lineendy=0;
    Myhalcv2::Mat i32_mXline,m_filter2,m32_filterIma;
    Myhalcv2::L_Point32 stepfindST,stepfindED;//结果线1拟合区域,(下方)
    Myhalcv2::L_Point32 midfindST,midfindED;//结果线2拟合区域,(上方)
    Int32 maxj,maxi;
    Int32 nihenum=0;
    Int32 nstarti,nendi,nstartj,nendj;
    Myhalcv2::L_Point32 linepoint32ST,linepoint32ED;
    Myhalcv2::L_line tileline;	//结果线2以及原图的线,(短的)
    Myhalcv2::L_line headline;	//结果线1以及原图的线,(短的)
    Myhalcv2::L_line32 tileline32;
    Myhalcv2::L_line32 headline32;
    Myhalcv2::L_Point32 resultfocal1,resultfocal2,resultfocal;//交点
    Int32 t;
    Int32 jiguangTop,jiguangDeep,jiguangLeft,jiguangRight;
    Int32 leijiwrite;
    Int32 centerj,centeri;
    Int32 tempi;
    Uint8 handianEn;
    Myhalcv2::L_Point32 temp_resultfocal1,temp_resultfocal2;
    Myhalcv2::MyConect ImageConect,ImageConectlong,ImageConectlongPX,Imageheadline;
    Myhalcv2::houghlineinfo headlinehough,tilelinehough;
    cv::Point cv_point_st,cv_point_ed;
    cv::Point2f cv_point;
    Int32 b_duanxianmoshi=0;//断线模式：1,下方线“压”上方线。0,上方线“压”下方
    Myhalcv2::L_Point32F faxian;
    Targetpoint targetpoint;

    /*********************/
    //算法参数
    Int32 pingjun=pm.als104_pingjun;//15;//二值阈值
    Int32 b_yanmofuzhu=pm.als104_b_yanmofuzhu;//1;//是否使用掩摸辅助
    Int32 b_gudingquyu=pm.als104_b_gudingquyu;//0;//是否固定区域
    Int32 widthliantongdis=pm.als104_widthliantongdis;//2;//激光宽度连通距离
    Int32 highliantongdis=pm.als104_highliantongdis;//15;//激光长度连通距离
    Int32 gujiaerzhi=pm.als104_gujiaerzhi;//160;//找骨架二值图
    Int32 jiguanghight=pm.als104_jiguanghight;//50;//整体激光最短长度
    Int32 jiguanglong=pm.als104_jiguanglong;//20;//单边激光最短长度
    Int32 jiguangkuandu=pm.als104_jiguangkuandu;//4;//激光粗细
    Int32 Updif=pm.als104_Updif;//0;//上半段倾斜开始斜度10
    Int32 Updifmin=pm.als104_Updifmin;//-5;//上半段倾斜终止斜度10
    Int32 Uplong=pm.als104_Uplong;//5;//上半段直线长度
    Int32 Downdif=pm.als104_Downdif;//0;//下半段倾斜开始斜度0
    Int32 Downdifmin=pm.als104_Downdifmin;//5;//下半段倾斜终止斜度0
    Int32 Downdlong=pm.als104_Downdlong;//5;//下半段直线长度
    Int32 dis_center_st=pm.als104_dis_center_st;//0;     //距离中心点此处后开始统计
    Int32 dis_center_ed=pm.als104_dis_center_ed;//500;  //距离中心点此处后停止统计
    Int32 b_KalmanFilter=pm.als104_b_KalmanFilter;//是否使用卡尔曼滤波
    float KalmanQF=pm.als104_KalmanQF/1000.0;//系统噪声方差矩阵Q 
    float KalmanRF=pm.als104_KalmanRF/1000.0;//系统噪声方差矩阵R 

#ifdef DEBUG_ALG
    int debug_alg=1;
    RCLCPP_INFO(this->get_logger(), "start alg104");
#endif
    if(step==2)
    {
      return 0;
    }
    imageIn=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff_image);
    Myhalcv2::CvMatToMat(cvimgIn,&imageIn,cv8uc1_Imagebuff_image);
    imageGasu=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff5);
    Myhalcv2::Mygausspyramid_2levl(imageIn,&imageGasu);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif    
    if(step!=0)
    {
      imageGasupain=MatCreatClone(imageGasu,cv8uc1_Imagebuff8);
    }
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==3)
    {
      Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
      return 0;
    }
    imageBry=Myhalcv2::MatCreat(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff4);
    Myhalcv2::Mynormalize(imageGasu,&imageBry);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==4)
    {
      Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
      return 0;
    }
    Myhalcv2::Mybinaryval(imageBry,&bryvalue,Myhalcv2::MHC_BARINYVAL_MEAN);
    i32_bryvalue=(Int32)bryvalue+pingjun;//求平均值二值化阈值
    Myhalcv2::Mybinary(imageBry,&imageBry,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==5)
    {
      Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
      return 0;
    }
    if(b_yanmofuzhu==1)
    {
        m_matMask=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff1);
        Myhalcv2::Myconnection2(imageBry,&ImageConect,jiguanghight,widthliantongdis,highliantongdis,Myhalcv2::MHC_MORPH_RECT,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//创建8联通区域ImageConect,最小面积120,两区域距离小于2认为同一区域
        Myhalcv2::Myselect_shape(&ImageConect,&ImageConectlong,Myhalcv2::MHC_CONNECT_WIDTH_HEIGHT,jiguanghight,MAX(ImageConect.nHeight,ImageConect.nWidth));
        Myhalcv2::Myregion_to_bin(&ImageConectlong,&m_matMask,255);
        Myhalcv2::Mydilation_circle2(m_matMask,&m_matMask,10,0,Myhalcv2::MHC_MORPH_RECT);
    }
    m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    if(b_gudingquyu==1)
    {
        if(firstsearch==1)
        {
            Myhalcv2::MyCutselfRoi(&imageGasu,firstsearch_stx,firstsearch_sty,firstsearch_edx-firstsearch_stx+1,firstsearch_edy-firstsearch_sty+1);
        }
    }
    if(b_yanmofuzhu==1)
    {
        Myhalcv2::Mynormalize_lineXY_mask(imageGasu,&m_brygujia,m_matMask,5);
    }
    else
    {
        Myhalcv2::Mynormalize_lineXY(imageGasu,&m_brygujia,5);
    }
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==6)
    {
      Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
      return 0;
    }
    i32_bryvalue=gujiaerzhi;
    Myhalcv2::Mybinary(m_brygujia,&m_brygujia,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==7)
    {
      Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
      return 0;
    }

    Myhalcv2::Myintersection(imageBry,m_brygujia,&imageBry);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==8)
    {
      Myhalcv2::Mymat_to_binself(&imageBry,255);
      Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
      return 0;
    }

    Myhalcv2::Myconnection2(imageBry,&ImageConect,jiguanghight,widthliantongdis,highliantongdis,Myhalcv2::MHC_MORPH_RECT,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//创建8联通区域ImageConect,最小面积120,两区域距离小于2认为同一区域
    Myhalcv2::Myselect_shape(&ImageConect,&ImageConect,Myhalcv2::MHC_CONNECT_WIDTH_HEIGHT,jiguanghight,MAX(ImageConect.nHeight,ImageConect.nWidth));
    Myhalcv2::Mysort_region(&ImageConect,&ImageConectlong,Myhalcv2::MHC_RIGHT_RIGHTTOLEFT_PAIXU);//在ImageConect中筛选出高度大于50的联通域
    if(ImageConectlong.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::Myselect_obj(&ImageConectlong,&ImageConectlongPX,0);
    Myhalcv2::Mysmallest_rectangle(&ImageConectlongPX,&jiguangLeft,&jiguangRight,&jiguangTop,&jiguangDeep);
    if(b_gudingquyu==1)
    {
        if(firstsearch==0)
        {
            firstsearch_stx=jiguangLeft-30;
            firstsearch_edx=jiguangRight+30;
            firstsearch_sty=jiguangTop-30;
            firstsearch_edy=jiguangDeep+30;
            if(firstsearch_stx<(Int32)imageGasu.startx)
            {
                firstsearch_stx=imageGasu.startx;
            }
            if(firstsearch_edx>(Int32)imageGasu.startx+imageGasu.width-1)
            {
                firstsearch_edx=imageGasu.startx+imageGasu.width-1;
            }
            if(firstsearch_sty<(Int32)imageGasu.starty)
            {
                firstsearch_sty=imageGasu.starty;
            }
            if(firstsearch_edy>(Int32)imageGasu.starty+imageGasu.height-1)
            {
                firstsearch_edy=imageGasu.starty+imageGasu.height-1;
            }
            firstsearch=1;
        }
        else
        {
            Int32 realstx,realsty,realedx,realedy;
            realstx=firstsearch_stx;
            realsty=firstsearch_sty;
            realedx=firstsearch_edx;
            realedy=firstsearch_edy;
            if(jiguangLeft>firstsearch_stx&&jiguangLeft<firstsearch_edx)
            {
                realstx=jiguangLeft;
            }
            if(jiguangRight>firstsearch_stx&&jiguangRight<firstsearch_edx)
            {
                realedx=jiguangRight;
            }
            if(jiguangTop>firstsearch_sty&&jiguangTop<firstsearch_edy)
            {
                realsty=jiguangTop;
            }
            if(jiguangDeep>firstsearch_sty&&jiguangDeep<firstsearch_edy)
            {
                realedy=jiguangDeep;
            }
            jiguangLeft=realstx;
            jiguangRight=realedx;
            jiguangTop=realsty;
            jiguangDeep=realedy;
        }
    }
    Myhalcv2::MyCutRoi(imageGasu,&m_tempmatIn,Myhalcv2::MHC_CUT_NOTCOPY,jiguangLeft,jiguangTop,jiguangRight-jiguangLeft+1,jiguangDeep-jiguangTop+1);
    Myhalcv2::Mynormalize_lineXY(m_tempmatIn,&imageBry,1);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==9)
    {
      Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
      return 0;
    }
    i32_bryvalue=gujiaerzhi;//求平均值二值化阈值
    Myhalcv2::Mybinary(imageBry,&imageBry,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==10)
    {
      Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
      return 0;
    }
    Myhalcv2::Myconnection2(imageBry,&ImageConect,jiguanghight,widthliantongdis,highliantongdis,Myhalcv2::MHC_MORPH_RECT,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//创建8联通区域ImageConect,最小面积120,两区域距离小于2认为同一区域
    Myhalcv2::Myselect_shape(&ImageConect,&ImageConectlong,Myhalcv2::MHC_CONNECT_WIDTH_HEIGHT,jiguanglong,MAX(ImageConect.nHeight,ImageConect.nWidth));//在ImageConect中筛选出高度大于50的联通域
    if(ImageConectlong.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::MyGetthinNoHough(&ImageConectlong,Myhalcv2::THIN_X,jiguangkuandu,&imageBry);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==11)
    {
      Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
      return 0;
    }
    Myhalcv2::Mydeleteconnection(imageBry,&imageBry,jiguanghight,highliantongdis,Myhalcv2::MHC_8LT);
#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "start alg104=%d",debug_alg++);
#endif 
    if(step==12)
    {
      Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
      return 0;
    }
    /***********************/
    //以下的图像几乎都是完美图像,需要检测出结果
    //以下对高斯图做卷积
    memset(X_line,0,sizeof(Int32)*nHeight);
    memset(X_lineMark,0,nHeight);

    X_Linestarty=0;
    X_Lineendy=0;

    if(step==13)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
    }
    //膨胀做
    Myhalcv2::Mydilation_circle2(imageBry,&imageBry,2,0,Myhalcv2::MHC_MORPH_RECT);

    if(b_gudingquyu==1)
    {
        nstartj=MAX(jiguangTop*4,0);
        nendj=MIN(jiguangDeep*4,nHeight-1);
        nstarti=MAX(jiguangLeft*4,0);
        nendi=MIN(jiguangRight*4,nWidth-1);
    }
    else
    {
        nstartj=MAX(jiguangTop*4,0);
        nendj=MIN(jiguangDeep*4,nHeight-1);
        nstarti=MAX(jiguangLeft*4-30,0);
        nendi=MIN(jiguangRight*4+30,nWidth-1);
    }
    Myhalcv2::MyCutRoi(imageIn,&m_tempmatIn,Myhalcv2::MHC_CUT_NOTCOPY,nstarti,nstartj,nendi-nstarti+1,nendj-nstartj+1);

    for(j=m_tempmatIn.starty;j<m_tempmatIn.starty+m_tempmatIn.height;j++)
    {
        Int32 sum_valuecoor=0;
        Int32 sum_value=0;

        for(i=m_tempmatIn.startx;i<m_tempmatIn.startx+m_tempmatIn.width;i++)
        {
            Int32 di=i>>2;
            Int32 dj=j>>2;
            if(imageBry.data[dj*imageBry.nWidth+di]!=0)
            {
                sum_valuecoor=sum_valuecoor+(Int32)m_tempmatIn.data[j*m_tempmatIn.nWidth+i]*i;
                sum_value=sum_value+m_tempmatIn.data[j*m_tempmatIn.nWidth+i];
            }
        }
        if(sum_value!=0)
        {
            X_line[j]=(float)sum_valuecoor/sum_value+0.5;
            if(X_Linestarty==0)
            {
                X_Linestarty=j;//骨架起点
            }
            X_Lineendy=j;//骨架终点
            X_lineMark[j]=1;
        }
        if(step==13)
        {
            if(sum_value!=0)
            {
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
            }
        }
    }
    if(step==13)
    {
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }
    if(X_Lineendy-X_Linestarty<50)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::Myfixdata(X_line,X_lineMark,nHeight);//修复空的线
    if(step==14)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        for(j=m_tempmatIn.starty;j<m_tempmatIn.starty+m_tempmatIn.height;j++)
        {
            imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
            imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
            imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
        }
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }

    maxi=RANDOM_MIN;
    for(j=X_Linestarty;j<=X_Lineendy;j++)
    {
        if(maxi<X_line[j])
        {
            maxi=X_line[j];
            maxj=j;
        }
    }
    i32_mXline=Myhalcv2::MatCreat(1,nHeight,Myhalcv2::CCV_32SC1,X_line);//把线横摆
    for(i=0;i<20;i++)
    {
        filterdata2[i]=1;
    }
    for(i=20;i<40;i++)
    {
        filterdata2[i]=-1;
    }
    m_filter2=Myhalcv2::MatCreat(1,40,Myhalcv2::CCV_16SC1,filterdata2);
    m32_filterIma=Myhalcv2::MatCreatzero(1,nHeight,Myhalcv2::CCV_32SC1,X_linedif32);
    Myhalcv2::Myfilter(i32_mXline,m_filter2,&m32_filterIma,Myhalcv2::CCV_32SC1,0,f_center);//卷积得到

    m_brygujia=Myhalcv2::MatCreatzero(3,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);  //创建上半截连通图

    for(j=X_Linestarty+24;j<=X_Lineendy-24;j++)
    {
        if(m32_filterIma.ptr_int[j]<=Updifmin&&m32_filterIma.ptr_int[j]>=Updif)
        {
            m_brygujia.data[1*m_brygujia.nWidth+j]=255;
        }
    }
    if(step==15)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        for(j=X_Linestarty+24;j<=X_Lineendy-24;j++)
        {
            if( m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
            {
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
            }
        }
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }
    Myhalcv2::Myconnection(m_brygujia,&ImageConect,Uplong,0,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);
    Myhalcv2::Mysort_region(&ImageConect,&ImageConectlong,Myhalcv2::MHC_LEFT_LEFTTORIGHT_PAIXU);
    if(ImageConectlong.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    stepfindST.y=ImageConectlong.AllMarkPoint[0].left;
    stepfindST.x=X_line[stepfindST.y];
    stepfindED.y=ImageConectlong.AllMarkPoint[ImageConectlong.AllMarkPointCount-1].right;
    stepfindED.x=X_line[stepfindED.y];

    m_brygujia=Myhalcv2::MatCreatzero(3,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);  //重新创建上半截连通图

    for(j=0;j<ImageConectlong.AllMarkPointCount;j++)
    {
        if(ImageConectlong.AllMarkPoint[j].left>maxj)
        {
            break;
        }
        for(i=ImageConectlong.AllMarkPoint[j].left;i<=ImageConectlong.AllMarkPoint[j].right;i++)
        {
            m_brygujia.data[1*m_brygujia.nWidth+i]=255;
            stepfindED.y=ImageConectlong.AllMarkPoint[j].right;
            stepfindED.x=X_line[stepfindED.y];
        }
    }

    if(step==16)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.y=(maxj>>2);
        linepoint32ST.x=(maxi>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        for(j=stepfindST.y;j<=stepfindED.y;j++)
        {
            if( m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
            {
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
            }
        }
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }

    /************************************/
    //人工辅助限制
    stepfindED.y=MIN(stepfindED.y,maxj-dis_center_st);
    if(stepfindED.y<(Int32)X_Linestarty+6)
        stepfindED.y=X_Linestarty+6;
    stepfindED.x=X_line[stepfindED.y];
    stepfindST.y=MAX(stepfindST.y,maxj-dis_center_ed);
    stepfindST.x=X_line[stepfindST.y];
    if(stepfindED.y-stepfindST.y<Uplong)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    if(step==17)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.y=(stepfindST.y>>2);
        linepoint32ST.x=(stepfindST.x>>2);
        linepoint32ED.y=(stepfindED.y>>2);
        linepoint32ED.x=(stepfindED.x>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ED,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        for(j=stepfindST.y;j<=stepfindED.y;j++)
        {
            if( m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
            {
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
            }
        }
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }
    /**************************************/
    nihenum=0;
    for(j=stepfindST.y;j<=stepfindED.y;j++)
    {
        if(m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
        {
            niheX[nihenum]=X_line[j];
            niheY[nihenum++]=j;
            cv::Point2f point(X_line[j],j);
            pointcloud.push_back(point);
        }
    }
    if(nihenum<Uplong)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::MyData_sqare_line(niheX,niheY,nihenum,nWidth,nHeight,Myhalcv2::MHC_MIXDIS_SQARE,&headline,&headlinehough);
    //找下半段

    m_brygujia=Myhalcv2::MatCreatzero(3,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);  //创建下半截连通图

    for(j=X_Linestarty+24;j<X_Lineendy-24;j++)
    {
        if(m32_filterIma.ptr_int[j]>=Downdifmin&&m32_filterIma.ptr_int[j]<=Downdif)
        {
            m_brygujia.data[1*m_brygujia.nWidth+j]=255;
        }
    }
    if(step==18)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        for(j=X_Linestarty+24;j<X_Lineendy-24;j++)
        {
            if( m_brygujia.data[1*m_brygujia.nWidth+j]==255)
            {
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
            }
        }
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }
    Myhalcv2::Myconnection(m_brygujia,&ImageConect,Downdlong,0,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);
    Myhalcv2::Mysort_region(&ImageConect,&ImageConectlong,Myhalcv2::MHC_LEFT_LEFTTORIGHT_PAIXU);
    if(ImageConectlong.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    midfindST.y=ImageConectlong.AllMarkPoint[0].left;
    midfindST.x=X_line[midfindST.y];
    midfindED.y=ImageConectlong.AllMarkPoint[ImageConectlong.AllMarkPointCount-1].right;
    midfindED.x=X_line[midfindED.y];

    m_brygujia=Myhalcv2::MatCreatzero(3,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);  //重新创建上半截连通图

    for(j=ImageConectlong.AllMarkPointCount-1;j>=0;j--)
    {
        if(ImageConectlong.AllMarkPoint[j].right<maxj)
        {
            break;
        }
        for(i=ImageConectlong.AllMarkPoint[j].left;i<=ImageConectlong.AllMarkPoint[j].right;i++)
        {
            m_brygujia.data[1*m_brygujia.nWidth+i]=255;
            midfindST.y=ImageConectlong.AllMarkPoint[j].left;
            midfindST.x=X_line[midfindST.y];
        }
    }
    if(step==19)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.y=(maxj>>2);
        linepoint32ST.x=(maxi>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        for(j=midfindST.y;j<=midfindED.y;j++)
        {
            if( m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
            {
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
            }
        }
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }

    /************************************/
    //人工辅助限制
    midfindST.y=MAX(midfindST.y,maxj+dis_center_st);
    if(midfindST.y>(Int32)(X_Lineendy-6))
        midfindST.y=X_Lineendy-6;
    midfindST.x=X_line[midfindST.y];
    midfindED.y=MIN(midfindED.y,maxj+dis_center_ed);
    midfindED.x=X_line[midfindED.y];
    if(midfindED.y-midfindST.y<Downdlong)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    /**************************************/
    if(step==20)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.y=(midfindST.y>>2);
        linepoint32ST.x=(midfindST.x>>2);
        linepoint32ED.y=(midfindED.y>>2);
        linepoint32ED.x=(midfindED.x>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ED,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        for(j=midfindST.y;j<=midfindED.y;j++)
        {
            if( m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
            {
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
            }
        }
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }
    nihenum=0;
    for(j=midfindST.y;j<=midfindED.y;j++)
    {
        if(m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
        {
            niheX[nihenum]=X_line[j];
            niheY[nihenum++]=j;
            cv::Point2f point(X_line[j],j);
            pointcloud.push_back(point);
        }
    }
    if(nihenum<Downdlong)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::MyData_sqare_line(niheX,niheY,nihenum,nWidth,nHeight,Myhalcv2::MHC_MIXDIS_SQARE,&tileline,&tilelinehough);
    if(step==21)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.y=(stepfindST.y>>2);
        linepoint32ST.x=(stepfindST.x>>2);
        linepoint32ED.y=(stepfindED.y>>2);
        linepoint32ED.x=(stepfindED.x>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,5,0,0,255,Myhalcv2::CV_CLRCLE_FILL);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ED,5,0,0,255,Myhalcv2::CV_CLRCLE_FILL);
        linepoint32ST.y=(midfindST.y>>2);
        linepoint32ST.x=(midfindST.x>>2);
        linepoint32ED.y=(midfindED.y>>2);
        linepoint32ED.x=(midfindED.x>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,5,255,255,0,Myhalcv2::CV_CLRCLE_FILL);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ED,5,255,255,0,Myhalcv2::CV_CLRCLE_FILL);
        Myhalcv2::MyPoint16to32(headline.st,&linepoint32ST);
        Myhalcv2::MyPoint16to32(headline.ed,&linepoint32ED);
        linepoint32ST.y=(linepoint32ST.y>>2);
        linepoint32ST.x=(linepoint32ST.x>>2);
        linepoint32ED.y=(linepoint32ED.y>>2);
        linepoint32ED.x=(linepoint32ED.x>>2);
        Myhalcv2::MyLine3col(&imageGasupain,linepoint32ST,linepoint32ED,255,0,0,Myhalcv2::CV_LINE_8LT,1);
        Myhalcv2::MyPoint16to32(tileline.st,&linepoint32ST);
        Myhalcv2::MyPoint16to32(tileline.ed,&linepoint32ED);
        linepoint32ST.y=(linepoint32ST.y>>2);
        linepoint32ST.x=(linepoint32ST.x>>2);
        linepoint32ED.y=(linepoint32ED.y>>2);
        linepoint32ED.x=(linepoint32ED.x>>2);
        Myhalcv2::MyLine3col(&imageGasupain,linepoint32ST,linepoint32ED,0,255,0,Myhalcv2::CV_LINE_8LT,1);
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }

    //求得两直线交点
    if(0!=Myhalcv2::MyGetLinefocal(headline,tileline,&resultfocal))
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
    #endif
        return 1;
    }
    
    Myline16to32(tileline,&tileline32);
    Myline16to32(headline,&headline32);

    Myhalcv2::MyPoint16to32(headline.st,&linepoint32ST);
    Myhalcv2::MyPoint16to32(tileline.st,&linepoint32ED);
    MyGetLinefocalBisection(resultfocal,linepoint32ST,linepoint32ED,&faxian);

    if(b_KalmanFilter==1)
    {
        if(b_firstKalmanFilter==0)
        {
                b_firstKalmanFilter=1;
                Myhalcv2::MyKalman2D_init(resultfocal.x,resultfocal.y,KalmanQF,KalmanRF);
        } 
        else
        {
                Int32 outx,outy;
                Myhalcv2::MyKalman2D_filter(resultfocal.x,resultfocal.y,&outx,&outy);
                resultfocal.x=outx;
                resultfocal.y=outy;
        }
    }

    if(step==1)
    {
        Myhalcv2::L_POINT32F f_temp;
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        if(cvimgIn.type()==CV_8UC1)
        cv::cvtColor(cvimgIn,cvimgIn,cv::COLOR_GRAY2BGR);
        cv_point_st.x=(headline32.st.x>>2);
        cv_point_st.y=(headline32.st.y>>2);
        cv_point_ed.x=(headline32.ed.x>>2);
        cv_point_ed.y=(headline32.ed.y>>2);
        cv::line(cvimgIn,cv_point_st,cv_point_ed,cv::Scalar(255,0,0),1);
        cv_point_st.x=(tileline32.st.x>>2);
        cv_point_st.y=(tileline32.st.y>>2);
        cv_point_ed.x=(tileline32.ed.x>>2);
        cv_point_ed.y=(tileline32.ed.y>>2);
        cv::line(cvimgIn,cv_point_st,cv_point_ed,cv::Scalar(0,255,0),1);
        cv_point_st.x=(resultfocal.x>>2);
        cv_point_st.y=(resultfocal.y>>2);
        cv::circle(cvimgIn,cv_point_st,5,cv::Scalar(0,0,255),1);
        f_temp.x=faxian.x*1000+resultfocal.x;
        f_temp.y=faxian.y*1000+resultfocal.y;
        cv_point_st.x=(resultfocal.x>>2);
        cv_point_st.y=(resultfocal.y>>2);
        cv_point_ed.x=(f_temp.x/4);
        cv_point_ed.y=(f_temp.y/4);
        cv::line(cvimgIn,cv_point_st,cv_point_ed,cv::Scalar(255,255,0),1);
    }
    solderjoints=false;
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

#ifdef DEBUG_ALG;
    RCLCPP_INFO(this->get_logger(), "finish alg104");
#endif 
    
    return 0;
}

}