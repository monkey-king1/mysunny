#include "laser_imagepos/laser_imagepos.hpp"

#include "opencv2/opencv.hpp"
namespace laser_imagepos
{

using rcl_interfaces::msg::SetParametersResult;

void LaserImagePos::alg107_declare_parameters()
{
    this->declare_parameter("als107_exposure_time", pm.als107_exposure_time);
    this->declare_parameter("als107_pingjun", pm.als107_pingjun);
    this->declare_parameter("als107_b_yanmofuzhu", pm.als107_b_yanmofuzhu);
    this->declare_parameter("als107_b_gudingquyu", pm.als107_b_gudingquyu);
    this->declare_parameter("als107_widthliantongdis", pm.als107_widthliantongdis);
    this->declare_parameter("als107_highliantongdis", pm.als107_highliantongdis);
    this->declare_parameter("als107_gujiaerzhi", pm.als107_gujiaerzhi);
    this->declare_parameter("als107_jiguanghight", pm.als107_jiguanghight);
    this->declare_parameter("als107_jiguanglong", pm.als107_jiguanglong);
    this->declare_parameter("als107_jiguangkuandu", pm.als107_jiguangkuandu);
    this->declare_parameter("als107_Updif", pm.als107_Updif);
    this->declare_parameter("als107_Updifmin", pm.als107_Updifmin);
    this->declare_parameter("als107_Uplong", pm.als107_Uplong);
    this->declare_parameter("als107_Downdif", pm.als107_Downdif);
    this->declare_parameter("als107_Downdifmin", pm.als107_Downdifmin);
    this->declare_parameter("als107_Downdlong", pm.als107_Downdlong);
    this->declare_parameter("als107_St_Down", pm.als107_St_Down);
    this->declare_parameter("als107_Ed_Down", pm.als107_Ed_Down);
    this->declare_parameter("als107_St_Up", pm.als107_St_Up);
    this->declare_parameter("als107_Ed_Up", pm.als107_Ed_Up);
    this->declare_parameter("als107_dis_center_st", pm.als107_dis_center_st);
    this->declare_parameter("als107_dis_center_st2", pm.als107_dis_center_st2);
    this->declare_parameter("als107_dis_center_ed2", pm.als107_dis_center_ed2);
    this->declare_parameter("als107_b_KalmanFilter", pm.als107_b_KalmanFilter); 
    this->declare_parameter("als107_KalmanQF", pm.als107_KalmanQF);
    this->declare_parameter("als107_KalmanRF", pm.als107_KalmanRF);
}

void LaserImagePos::alg107_update_parameters()
{
  const auto & vp = this->get_parameters(KEYS_ALS107);
  for (const auto & p : vp) {
    if (p.get_name() == "als107_exposure_time") {
      pm.als107_exposure_time = p.as_int();
    } else if (p.get_name() == "als107_pingjun") {
      pm.als107_pingjun = p.as_int();
    }
    else if (p.get_name() == "als107_b_yanmofuzhu") {
      pm.als107_b_yanmofuzhu = p.as_int();
    }
    else if (p.get_name() == "als107_b_gudingquyu") {
      pm.als107_b_gudingquyu = p.as_int();
    }
    else if (p.get_name() == "als107_widthliantongdis") {
      pm.als107_widthliantongdis = p.as_int();
    }
    else if (p.get_name() == "als107_highliantongdis") {
      pm.als107_highliantongdis = p.as_int();
    }
    else if (p.get_name() == "als107_gujiaerzhi") {
      pm.als107_gujiaerzhi = p.as_int();
    }
    else if (p.get_name() == "als107_jiguanghight") {
      pm.als107_jiguanghight = p.as_int();
    }
    else if (p.get_name() == "als107_jiguanglong") {
      pm.als107_jiguanglong = p.as_int();
    }
    else if (p.get_name() == "als107_jiguangkuandu") {
      pm.als107_jiguangkuandu = p.as_int();
    }
    else if (p.get_name() == "als107_Updif") {
      pm.als107_Updif = p.as_int();
    }
    else if (p.get_name() == "als107_Updifmin") {
      pm.als107_Updifmin = p.as_int();
    }
    else if (p.get_name() == "als107_Uplong") {
      pm.als107_Uplong = p.as_int();
    }
    else if (p.get_name() == "als107_Downdif") {
      pm.als107_Downdif = p.as_int();
    }
    else if (p.get_name() == "als107_Downdifmin") {
      pm.als107_Downdifmin = p.as_int();
    }
    else if (p.get_name() == "als107_Downdlong") {
      pm.als107_Downdlong = p.as_int();
    }
    else if (p.get_name() == "als107_St_Down") {
      pm.als107_St_Down = p.as_int();
    }
    else if (p.get_name() == "als107_Ed_Down") {
      pm.als107_Ed_Down = p.as_int();
    }
    else if (p.get_name() == "als107_St_Up") {
      pm.als107_St_Up = p.as_int();
    }
    else if (p.get_name() == "als107_Ed_Up") {
      pm.als107_Ed_Up = p.as_int();
    }
    else if (p.get_name() == "als107_dis_center_st") {
      pm.als107_dis_center_st = p.as_int();
    }
    else if (p.get_name() == "als107_dis_center_st2") {
      pm.als107_dis_center_st2 = p.as_int();
    }
    else if (p.get_name() == "als107_dis_center_ed2") {
      pm.als107_dis_center_ed2 = p.as_int();
    }
    else if (p.get_name() == "als107_b_KalmanFilter") {
      pm.als107_b_KalmanFilter = p.as_int();
    }
    else if (p.get_name() == "als107_KalmanQF") {
      pm.als107_KalmanQF = p.as_int();
    }
    else if (p.get_name() == "als107_KalmanRF") {
      pm.als107_KalmanRF = p.as_int();
    }
  }
}

int LaserImagePos::alg107_getcallbackParameter(const rclcpp::Parameter &p)
{
    if (p.get_name() == "als107_exposure_time") {
        auto k = p.as_int();
        pm.als107_exposure_time=k;
            if(pm.task_num==107){
                _param_camera->set_parameters({rclcpp::Parameter("exposure_time", pm.als107_exposure_time)});}
            return 1;}
    else if(p.get_name() == "als107_pingjun") {
        auto k = p.as_int();
        pm.als107_pingjun=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_b_yanmofuzhu") {
        auto k = p.as_int();
        pm.als107_b_yanmofuzhu=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_b_gudingquyu") {
        auto k = p.as_int();
        pm.als107_b_gudingquyu=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_widthliantongdis") {
        auto k = p.as_int();
        pm.als107_widthliantongdis=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_highliantongdis") {
        auto k = p.as_int();
        pm.als107_highliantongdis=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_gujiaerzhi") {
        auto k = p.as_int();
        pm.als107_gujiaerzhi=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_jiguanghight") {
        auto k = p.as_int();
        pm.als107_jiguanghight=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_jiguanglong") {
        auto k = p.as_int();
        pm.als107_jiguanglong=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_jiguangkuandu") {
        auto k = p.as_int();
        pm.als107_jiguangkuandu=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_Updif") {
        auto k = p.as_int();
        pm.als107_Updif=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_Updifmin") {
        auto k = p.as_int();
        pm.als107_Updifmin=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_Uplong") {
        auto k = p.as_int();
        pm.als107_Uplong=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_Downdif") {
        auto k = p.as_int();
        pm.als107_Downdif=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_Downdifmin") {
        auto k = p.as_int();
        pm.als107_Downdifmin=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_Downdlong") {
        auto k = p.as_int();
        pm.als107_Downdlong=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_St_Down") {
        auto k = p.as_int();
        pm.als107_St_Down=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_Ed_Down") {
        auto k = p.as_int();
        pm.als107_Ed_Down=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_St_Up") {
        auto k = p.as_int();
        pm.als107_St_Up=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_Ed_Up") {
        auto k = p.as_int();
        pm.als107_Ed_Up=p.as_int();
            return 1;}   
    else if(p.get_name() == "als107_dis_center_st") {
        auto k = p.as_int();
        pm.als107_dis_center_st=p.as_int();
            return 1;}   
    else if(p.get_name() == "als107_dis_center_st2") {
        auto k = p.as_int();
        pm.als107_dis_center_st2=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_dis_center_ed2") {
        auto k = p.as_int();
        pm.als107_dis_center_ed2=p.as_int();
            return 1;}                      
    else if(p.get_name() == "als107_b_KalmanFilter") {
        auto k = p.as_int();
        pm.als107_b_KalmanFilter=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_KalmanQF") {
        auto k = p.as_int();
        pm.als107_KalmanQF=p.as_int();
            return 1;}
    else if(p.get_name() == "als107_KalmanRF") {
        auto k = p.as_int();
        pm.als107_KalmanRF=p.as_int();
            return 1;}  
    return 0;
}

int LaserImagePos::alg107_runimage( cv::Mat &cvimgIn,
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
    Int32 zhengshunum=0;
    Int32 stepfind=0;
    Myhalcv2::L_Point32 stepfindST,stepfindED;//结果线1拟合区域,(下方)
    Myhalcv2::L_Point32 midfindST,midfindED;//结果线2拟合区域,(上方)
    Int32 cutWSize=40;
    Int32 cutHSize=250;
    Int32 latsj;
    Int32 nihenum=0;
    Myhalcv2::L_Point32 linepoint32ST,linepoint32ED;
    Myhalcv2::L_line tileline;	//结果线2以及原图的线,(短的)
    Myhalcv2::L_line headline;	//结果线1以及原图的线,(短的)
    Myhalcv2::L_line endline;	//结果线1以及原图的线,(短的)
    Myhalcv2::L_Point32 resultfocal1,resultfocal,resultfocal2,resultfocal3;//交点
    Int32 jiguangTop,jiguangDeep,jiguangLeft,jiguangRight;
    Myhalcv2::MyConect ImageConect,ImageConectlong,ImageConectlongPX;
    Myhalcv2::houghlineinfo headlinehough,tilelinehough;
    cv::Point cv_point_st,cv_point_ed;
    cv::Point2f cv_point;
    Myhalcv2::L_Point32F faxian,faxian3;
    Int32 nstarti,nendi,nstartj,nendj;
    Targetpoint targetpoint;

    /*********************/
    //算法参数
    Int32 pingjun=pm.als107_pingjun;//二值阈值
    Int32 b_yanmofuzhu=pm.als107_b_yanmofuzhu;//是否使用掩摸辅助
    Int32 b_gudingquyu=pm.als107_b_gudingquyu;//是否固定区域
    Int32 widthliantongdis=pm.als107_widthliantongdis;//激光宽度连通距离
    Int32 highliantongdis=pm.als107_highliantongdis;//激光长度连通距离
    Int32 gujiaerzhi=pm.als107_gujiaerzhi;//找骨架二值图
    Int32 jiguanghight=pm.als107_jiguanghight;//整体激光最短长度
    Int32 jiguanglong=pm.als107_jiguanglong;//单边激光最短长度
    Int32 jiguangkuandu=pm.als107_jiguangkuandu;//激光粗细
    Int32 Updif=pm.als107_Updif;//上半段倾斜开始斜度10
    Int32 Updifmin=pm.als107_Updifmin;//上半段倾斜终止斜度10
    Int32 Uplong=pm.als107_Uplong;//上半段直线长度
    Int32 Downdif=pm.als107_Downdif;//下半段倾斜开始斜度0
    Int32 Downdifmin=pm.als107_Downdifmin;//下半段倾斜终止斜度0
    Int32 Downdlong=pm.als107_Downdlong;//下半段直线长度
    Int32 St_Down=pm.als107_St_Down;//下半段拟合起点
    Int32 Ed_Down=pm.als107_Ed_Down;//下半段拟合终点
    Int32 St_Up=pm.als107_St_Up;//上半段拟合起点
    Int32 Ed_Up=pm.als107_Ed_Up;//上半段拟合终点
    Int32 dis_center_st=pm.als107_dis_center_st;     //距离中心点此处后开始统计
    Int32 dis_center_st2=pm.als107_dis_center_st2;//5;//0;     //距离中心点此处后开始统计
    Int32 dis_center_ed2=pm.als107_dis_center_ed2;//100;//30;  //距离中心点此处后停止统计
    Int32 b_KalmanFilter=pm.als107_b_KalmanFilter;//是否使用卡尔曼滤波
    float KalmanQF=pm.als107_KalmanQF/1000.0;//系统噪声方差矩阵Q 
    float KalmanRF=pm.als107_KalmanRF/1000.0;//系统噪声方差矩阵R 
    
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
    imageBry=Myhalcv2::MatCreat(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff4);
    Myhalcv2::Mynormalize(imageGasu,&imageBry);
    if(step==4)
    {
        Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
        return 0;
    }
    Myhalcv2::Mybinaryval(imageBry,&bryvalue,Myhalcv2::MHC_BARINYVAL_MEAN);
    i32_bryvalue=(Int32)bryvalue+pingjun;//求平均值二值化阈值
    Myhalcv2::Mybinary(imageBry,&imageBry,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);
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
    if(step==6)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }
    i32_bryvalue=gujiaerzhi;
    Myhalcv2::Mybinary(m_brygujia,&m_brygujia,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);
    if(step==7)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }
    Myhalcv2::Myintersection(imageBry,m_brygujia,&imageBry);

    if(step==8)
    {
        Myhalcv2::Mymat_to_binself(&imageBry,255);
        Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
        return 0;
    }

    Myhalcv2::Myconnection2(imageBry,&ImageConect,jiguanghight,widthliantongdis,highliantongdis,Myhalcv2::MHC_MORPH_RECT,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//创建8联通区域ImageConect,最小面积120,两区域距离小于2认为同一区域
    Myhalcv2::Myselect_shape(&ImageConect,&ImageConect,Myhalcv2::MHC_CONNECT_WIDTH_HEIGHT,jiguanghight,MAX(ImageConect.nHeight,ImageConect.nWidth));
    Myhalcv2::Mysort_region(&ImageConect,&ImageConectlong,Myhalcv2::MHC_TOPTOBOTTOM_PAIXU);//在ImageConect中筛选出高度大于50的联通域
    if(ImageConectlong.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::Mysort_region(&ImageConect,&ImageConectlongPX,Myhalcv2::MHC_HEIGHT_PAIXU);//在ImageConect中筛选出高度大于50的联通域
    if(ImageConectlongPX.AllMarkPoint[ImageConectlongPX.AllMarkPointCount-1].bottom!=
        ImageConectlong.AllMarkPoint[ImageConectlong.AllMarkPointCount-1].bottom)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::Myselect_obj(&ImageConectlong,&ImageConectlongPX,ImageConectlong.AllMarkPointCount-1);
    Myhalcv2::Mysmallest_rectangle(&ImageConectlongPX,&jiguangLeft,&jiguangRight,&jiguangTop,&jiguangDeep);
    if(b_gudingquyu==1)
    {
        if(firstsearch==0)
        {
            firstsearch_stx=jiguangLeft-30;
            firstsearch_edx=jiguangRight+30;
            firstsearch_sty=jiguangTop-30;
            firstsearch_edy=jiguangDeep+50;
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
    else
    {
        firstsearch_stx=jiguangLeft-30;
        firstsearch_edx=jiguangRight+30;
        if(firstsearch_stx<(Int32)imageGasu.startx)
        {
            firstsearch_stx=imageGasu.startx;
        }
        if(firstsearch_edx>(Int32)imageGasu.startx+imageGasu.width-1)
        {
            firstsearch_edx=imageGasu.startx+imageGasu.width-1;
        }
        jiguangLeft=firstsearch_stx;
        jiguangRight=firstsearch_edx;
    }

    Myhalcv2::MyCutRoi(imageGasu,&m_tempmatIn,Myhalcv2::MHC_CUT_NOTCOPY,jiguangLeft,jiguangTop,jiguangRight-jiguangLeft+1,jiguangDeep-jiguangTop+1);
    Myhalcv2::Mynormalize_lineXY(m_tempmatIn,&imageBry,1);
    if(step==9)
    {
        Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
        return 0;
    }
    i32_bryvalue=gujiaerzhi;//求平均值二值化阈值
    Myhalcv2::Mybinary(imageBry,&imageBry,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);

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

    Myhalcv2::MyGetlanothinNoHough(imageBry,Myhalcv2::THIN_X,jiguangkuandu,&imageBry);

    if(0!=Myhalcv2::Mydeleteconnection(imageBry,&imageBry,jiguanghight,highliantongdis,Myhalcv2::MHC_8LT))
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    if(step==11)
    {
        Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
        return 0;
    }
    if(step==12)
    {
        Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
        return 0;
    }
    //以下的图像几乎都是完美图像,需要检测出结果
    //以下求灰度质心
    memset(X_line,0,sizeof(Int32)*nHeight);
    memset(X_lineMark,0,nHeight);

    X_Linestarty=0;
    X_Lineendy=0;
    //以下取出二值图结果中每行卷积最大值

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
            float p=(float)sum_valuecoor/sum_value;
            X_line[j]=p+0.5;
            if(X_Linestarty==0)
            {
                X_Linestarty=j;//骨架起点
            }
            X_Lineendy=j;//骨架终点
            X_lineMark[j]=1;

            cv::Point2f point(p,j);
            pointcloud.push_back(point);
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

    if(St_Down+Ed_Down+St_Up+Ed_Up>X_Lineendy-X_Linestarty+1)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }

    //找下半段
    m_brygujia=Myhalcv2::MatCreatzero(3,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);  //创建下半截连通图

    for(j=X_Linestarty+24;j<X_Lineendy-24;j++)
    {
        if(m32_filterIma.ptr_int[j]<=Downdifmin&&m32_filterIma.ptr_int[j]>=Downdif)
        {
            m_brygujia.data[1*m_brygujia.nWidth+j]=255;
        }
    }
    if(step==15)
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

    latsj=MAX(X_Lineendy-Ed_Down,X_Linestarty+24);


    midfindST.y=ImageConectlong.AllMarkPoint[0].left;
    midfindST.x=X_line[midfindST.y];
    midfindED.y=ImageConectlong.AllMarkPoint[ImageConectlong.AllMarkPointCount-1].right;
    midfindED.x=X_line[midfindED.y];

    m_brygujia=Myhalcv2::MatCreatzero(3,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);  //重新创建下半截连通图

    for(j=ImageConectlong.AllMarkPointCount-1;j>=0;j--)
    {
        for(i=ImageConectlong.AllMarkPoint[j].left;i<=ImageConectlong.AllMarkPoint[j].right;i++)
        {
            m_brygujia.data[1*m_brygujia.nWidth+i]=255;
        }
        if(ImageConectlong.AllMarkPoint[j].left<latsj)
        {
            midfindST.y=ImageConectlong.AllMarkPoint[j].left;
            midfindST.x=X_line[midfindST.y];
            break;
        }
    }
    if(step==16)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.y=(latsj>>2);
        linepoint32ST.x=(X_line[latsj]>>2);
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

    for(j=midfindST.y;j<=MIN(midfindST.y+dis_center_st,midfindED.y);j++)
    {
        m_brygujia.data[1*m_brygujia.nWidth+j]=0;
    }
    midfindST.y=MIN(midfindST.y+dis_center_st,midfindED.y);
    midfindST.x=X_line[midfindST.y];

    nihenum=0;
    for(j=midfindST.y;j<=midfindED.y;j++)
    {
        if(m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
        {
            niheX[nihenum]=X_line[j];
            niheY[nihenum++]=j;
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

    m_matMask=Myhalcv2::MatCreatClone(m_brygujia,cv8uc1_Imagebuff1);

    if(step==17)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.x=(midfindST.x>>2);
        linepoint32ST.y=(midfindST.y>>2);
        linepoint32ED.x=(midfindED.x>>2);
        linepoint32ED.y=(midfindED.y>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ED,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        for(j=0;j<nihenum;j++)
        {
            imageGasupain.ptr_Vec3b[(niheY[j]>>2)*imageGasupain.nWidth+(niheX[j]>>2)].data1=0;
            imageGasupain.ptr_Vec3b[(niheY[j]>>2)*imageGasupain.nWidth+(niheX[j]>>2)].data2=0;
            imageGasupain.ptr_Vec3b[(niheY[j]>>2)*imageGasupain.nWidth+(niheX[j]>>2)].data3=255;
        }
        Myhalcv2::MyPoint16to32(tileline.st,&linepoint32ST);
        Myhalcv2::MyPoint16to32(tileline.ed,&linepoint32ED);
        linepoint32ST.x=(linepoint32ST.x>>2);
        linepoint32ST.y=(linepoint32ST.y>>2);
        linepoint32ED.x=(linepoint32ED.x>>2);
        linepoint32ED.y=(linepoint32ED.y>>2);
        Myhalcv2::MyLine3col(&imageGasupain,linepoint32ST,linepoint32ED,255,0,0,Myhalcv2::CV_LINE_8LT,1);
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }

    //找上半段
    m_brygujia=Myhalcv2::MatCreatzero(3,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);  //创建上半截连通图

    for(j=X_Linestarty+24;j<=midfindST.y;j++)
    {
        if(m32_filterIma.ptr_int[j]>=Updifmin&&m32_filterIma.ptr_int[j]<=Updif)
        {
            m_brygujia.data[1*m_brygujia.nWidth+j]=255;
        }
    }
    if(step==18)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        for(j=X_Linestarty+24;j<=midfindST.y;j++)
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
    stepfindST.y=ImageConectlong.AllMarkPoint[ImageConectlong.AllMarkPointCount-1].left;
    stepfindST.x=X_line[stepfindST.y];
    stepfindED.y=ImageConectlong.AllMarkPoint[ImageConectlong.AllMarkPointCount-1].right;
    stepfindED.x=X_line[stepfindED.y];

   
    //检查如果最右边的不是最大的连通就报警
    Myhalcv2::Mysort_region(&ImageConect,&ImageConectlongPX,Myhalcv2::MHC_MIANJI_PAIXU);
    if(ImageConectlongPX.AllMarkPoint[ImageConectlongPX.AllMarkPointCount-1].right!=stepfindED.y)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }

    if(step==19)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.x=(stepfindST.x>>2);
        linepoint32ST.y=(stepfindST.y>>2);
        linepoint32ED.x=(stepfindED.x>>2);
        linepoint32ED.y=(stepfindED.y>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ED,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        for(j=stepfindST.y;j<=stepfindED.y;j++)
        {
            if(m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
            {
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data1=255;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data2=0;
                imageGasupain.ptr_Vec3b[(j>>2)*imageGasupain.nWidth+(X_line[j]>>2)].data3=0;
            }
        }
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }

    m_brygujia=Myhalcv2::MatCreatzero(3,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);  //重新创建上半截连通图
    for(j=0;j<ImageConectlong.AllMarkPoint[ImageConectlong.AllMarkPointCount-1].PointArea;j++)
    {
        m_brygujia.data[1*m_brygujia.nWidth+ImageConectlong.AllMarkPoint[ImageConectlong.AllMarkPointCount-1].point[j].x]=1;
    }

    for(j=stepfindST.y;j<MAX(stepfindED.y-dis_center_ed2,stepfindST.y);j++)
    {
        m_brygujia.data[1*m_brygujia.nWidth+j]=0;
    }
    stepfindST.y=MAX(stepfindED.y-dis_center_ed2,stepfindST.y);
    stepfindST.x=X_line[stepfindST.y];

    for(j=stepfindED.y;j>MAX(stepfindST.y,stepfindED.y-dis_center_st2);j--)
    {
        m_brygujia.data[1*m_brygujia.nWidth+j]=0;
    }
    stepfindED.y=MAX(stepfindST.y,stepfindED.y-dis_center_st2);
    stepfindED.x=X_line[stepfindED.y];
    nihenum=0;
    for(j=stepfindST.y;j<stepfindED.y;j++)
    {
        if(m_brygujia.data[1*m_brygujia.nWidth+j]!=0)
        {
            niheX[nihenum]=X_line[j];
            niheY[nihenum++]=j;
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
    if(step==20)
    {
        Myhalcv2::MatClone(imageGasu,&imageGasupain);
        Myhalcv2::MyBRYtoRGB(imageGasupain,&imageGasupain);
        linepoint32ST.x=(stepfindST.x>>2);
        linepoint32ST.y=(stepfindST.y>>2);
        linepoint32ED.x=(stepfindED.x>>2);
        linepoint32ED.y=(stepfindED.y>>2);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ST,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        Myhalcv2::MyCircle3col(&imageGasupain,linepoint32ED,4,0,255,0,Myhalcv2::CV_CLRCLE_FILL);
        for(j=0;j<nihenum;j++)
        {
            imageGasupain.ptr_Vec3b[(niheY[j]>>2)*imageGasupain.nWidth+(niheX[j]>>2)].data1=0;
            imageGasupain.ptr_Vec3b[(niheY[j]>>2)*imageGasupain.nWidth+(niheX[j]>>2)].data2=0;
            imageGasupain.ptr_Vec3b[(niheY[j]>>2)*imageGasupain.nWidth+(niheX[j]>>2)].data3=255;
        }
        Myhalcv2::MyPoint16to32(headline.st,&linepoint32ST);
        Myhalcv2::MyPoint16to32(headline.ed,&linepoint32ED);
        Myhalcv2::MyLine3col(&imageGasupain,linepoint32ST,linepoint32ED,255,0,0,Myhalcv2::CV_LINE_8LT,1);
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }

    if(headlinehough.theta<MHC_TETARANGE/2)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }

    if(0!=Myhalcv2::MyGetLinefocal(headline,tileline,&resultfocal))
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    if(resultfocal.x<0||resultfocal.x>=nWidth||resultfocal.y<0||resultfocal.y>=nHeight)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::MyPoint16to32(headline.st,&linepoint32ST);
    Myhalcv2::MyPoint16to32(tileline.ed,&linepoint32ED);
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
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        if(cvimgIn.type()==CV_8UC1)
            cv::cvtColor(cvimgIn,cvimgIn,cv::COLOR_GRAY2BGR);
        cv_point_st.x=(headline.st.x>>2);
        cv_point_st.y=(headline.st.y>>2);
        cv_point_ed.x=(headline.ed.x>>2);
        cv_point_ed.y=(headline.ed.y>>2);
        cv::line(cvimgIn,cv_point_st,cv_point_ed,cv::Scalar(255,0,0),1);
        cv_point_st.x=(tileline.st.x>>2);
        cv_point_st.y=(tileline.st.y>>2);
        cv_point_ed.x=(tileline.ed.x>>2);
        cv_point_ed.y=(tileline.ed.y>>2);
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
    return 0;
}

}