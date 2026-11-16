/*
 * find.c
 *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
 *  Created on: 2025��4��26��
 *      Author: 39653
 */
#include "find.h"
#include "image.h"
#include "string.h"
#include <math.h>
#include "zf_device_mt9v03x.h"
#include "globals.h"
uint8 mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W];
uint8 display_image[MT9V03X_H][MT9V03X_W];
uint8_t labelBuffer[MT9V03X_W*MT9V03X_H]={0};//��Ǿ���
float QQQQ;
float count19;
float count20;
float count21;

float screen_x;
float screen_y;
int why;
int xx=0;//��¼X����
int yy=0;//��¼������
void Set_image_twovalues(uint8 value)   ////���ö�ֵ������ֵ��
{
    for(uint8 i=0;i<MT9V03X_H;i++)
    {

        for(uint8 j=0;j<MT9V03X_W;j++)
        {

                 // ��ֵ������
                  mt9v03x_image_BandW[i][j] = (mt9v03x_image[i][j] >= value) ? 1 : 0;
}
}
                  //erosion_operation(mt9v03x_image_BandW);//��ʴһ��
                  //PENGZHANG_operation(mt9v03x_image_BandW);
                 // ת��Ϊ��ʾ��ʽ
      for(uint8 l=0;l<MT9V03X_H;l++)
        {
           for(uint8 k=0;k<MT9V03X_W;k++)
              {
                  display_image[l][k] = mt9v03x_image_BandW[l][k] * 255;
              }
        }
}

volatile uint8_t dotcnt=0;
volatile uint8_t dotlie[188];//����Ҫ�����������Ļ���Ұ׵���У��������û�쵽�׵�ͻ�ʧ��
volatile uint8_t dothang[120];//ͬ��
volatile short sumlie=0,sumhang=0;


void Seek_Beacon ()//�ҵƳ���
{
  uint8_t tm=0;
  uint8_t nr=0; //��
  uint8_t nc=0; //��

  dotcnt=0;
  for (nr = 1; nr <MT9V03X_H - 1; nr++)
  {
    for (nc = 1; nc <MT9V03X_W - 1; nc++)
    {
      // 188*120�ֱ����£������������׵㣬����Ϊ�ǿ��ܵ��ű��
      if ((mt9v03x_image_BandW[nr - 1][nc] +mt9v03x_image_BandW[nr + 1][nc] +mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] > 1))
      {
        dothang[dotcnt]=nr;     // ��¼���еİ׵�ˮƽ����λ��Ҳ��������ƫ��
        dotlie[dotcnt++]=nc;    // ��¼���еİ׵�ˮ��ֱ��λ��Ҳ����Զ��
      }
    }
  }
  if(dotcnt) // �����аװ�
  {
    sumlie=0,sumhang=0;         // ����ϴεĽ��
    for(tm=0;tm<dotcnt;tm++)
    {
      sumlie+=dotlie[tm];     // ���а׵�����ƫ��ֵ���
      sumhang+=dothang[tm];   // ���а׵�����ƫ��ֵ��ͣ���ʱû�õ�
    }
    sumlie =sumlie/dotcnt;      // �Ƶ��������ĵ�
    sumhang=sumhang/dotcnt;     // �Ƶ�Զ�����ĵ㣬��ʱû�õ�
  }

  ips200_draw_line(0,sumhang,188,sumhang,RGB565_RED);
  ips200_draw_line(sumlie,0,sumlie,120,RGB565_RED);

  int8_t comparative=MT9V03X_W/2-sumlie;
  //ips200_show_int(150,150,sumhang,8);
  ips200_show_int(150,150,comparative,8);
  return;
}



int16_t OFFSET0;    // ��Զ������������ֵ�ۺ�ƫ����
int16_t OFFSET1;    // �ڶ���
int16_t OFFSET2;    // �����������
int area_map[256]={0};


/*****************************************************************************
*�������ƣ�void SearchNeighbor_4(unsigned char *bitmap, unsigned char *labelmap,uint16 labelIndex,uint16 pixelIndex, Queue *queue)
*����˵��������������������һ�����ӵ㣨pixelIndex����������������������ĸ��������أ����������������ؼ��뵱ǰ��ͨ�򣬲��ö��й������������أ�
*       �����ڶ�ֵͼ�񣨺ڰ�ͼ�����ҳ�������ͨ�İ�ɫ����Ŀ�꣩��������ÿ����ͨ������������λ�ú������ܺ�
*����˵����bitmap:�����ͼ��  labelmap:��Ǿ���
*�������أ���
*�޸�ʱ�䣺2022��4��3��
*��    ע��
******************************************************************************/
uint16_t area;
int Connected_x[256];
int Connected_y[256];
short Connected_xmin[256];
short Connected_ymin[256];
short Connected_xmax[256];
short Connected_ymax[256];
int32_t Connected_light[256];//累计连通域内所有像素的亮度值，用于计算平均亮度。

//����������
static int NeighborDirection[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};  //�������ƫ�������ҡ��¡����ϣ�
void SearchNeighbor_4(unsigned char *bitmap,
                        unsigned char *labelmap,
                        uint16_t labelIndex,
                        int pixelIndex,
                        Queue *queue,
                        int x, int y,
                        unsigned char *lightmap)
{
  int searchIndex, i, length;
  int qx, qy;
  int curCol;       // 当前像素的列坐标，用于边界检查
  int neighborCol;  // 邻居像素的列坐标

  labelmap[pixelIndex] = labelIndex;
  length = MT9V03X_W * MT9V03X_H;
  curCol = pixelIndex % MT9V03X_W;  // 计算当前像素所在的列

  for (i = 0; i < 4; i++)
  {
    // ★ 关键修复：先检查列边界，防止左右邻居环绕到上一行/下一行
    neighborCol = curCol + NeighborDirection[i][1];  // NeighborDirection[i][1] 是列偏移
    if (neighborCol < 0 || neighborCol >= MT9V03X_W)
    {
      continue;  // 越过图像左右边界，跳过
    }

    // 计算一维索引
    searchIndex = pixelIndex + NeighborDirection[i][0] * MT9V03X_W + NeighborDirection[i][1];

    // 此时 searchIndex 的行边界检查仍然有效（防止上下越界）
    if (searchIndex >= 0 && searchIndex < length
        && bitmap[searchIndex] == 1
        && labelmap[searchIndex] == 0)
    {
      labelmap[searchIndex] = labelIndex;
      area++;

      qy = searchIndex / MT9V03X_W;
      qx = searchIndex % MT9V03X_W;

      Connected_x[labelIndex] += qx;
      Connected_y[labelIndex] += qy;
      Connected_light[labelIndex] += lightmap[searchIndex];

      if(qy > Connected_ymax[labelIndex])
          Connected_ymax[labelIndex] = qy;
      if(qy < Connected_ymin[labelIndex])
          Connected_ymin[labelIndex] = qy;
      if(qx > Connected_xmax[labelIndex])
          Connected_xmax[labelIndex] = qx;
      if(qx < Connected_xmin[labelIndex])
          Connected_xmin[labelIndex] = qx;

      if (area >= 65535)
      {
        area = 65535;
        break;
      }
      PushQueue(queue, searchIndex);
    }
  }
}


/*****************************************************************************
*�������ƣ�void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char * labelmap)
*����˵������ͨ�����,���ű���ж�
*       �����ڶ�ֵͼ����ʶ�𲢱�����ж�������ͨ����Ŀ�꣩��ͬʱ����ÿ���������������������ƽ������
*����˵����bitmap:�����ͼ��  labelmap:��Ǿ���
*�������أ������ܵ��������
*�޸�ʱ�䣺2022��4��3��
*��    ע����
*ʹ��˵����ֻ�����������鼴�ɣ�i��2��ʼ
          area_map[i]Ϊÿ��������������
          Connected_x[i],Connected_y[i],Ϊÿ���������������
          Connected_light[i]��Ϊÿ�����������ƽ��ֵ
******************************************************************************/
//��Ƶ���ͨ�����ʮһ������
uint16_t areamax=0;//最大连通域的目标连通域大小
uint8_t labelmax=0,//最大连通域标签
        labeldel[256]={0}, //标记连通域小于3的连通域标签
        labelcount=0;  //总的连通域个数

//bitmap:二值图像  labelmap:标记矩阵
void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char *labelmap)
{
  uint16_t i, j, labelIndex = 1, del = 0, x = 1;
  int index, popIndex;
  // ★ 关键修复：使用栈上的固定大小循环队列，不使用 malloc，避免堆耗尽卡死
  Queue queue;
  queue.head = 0;
  queue.tail = 0;
  queue.count = 0;

  memset(labelmap, 0, MT9V03X_W * MT9V03X_H);
  memset(labeldel, 0, sizeof(labeldel));
  memset(area_map, 0, sizeof(area_map));
  memset(Connected_x, 0, sizeof(Connected_x));
  memset(Connected_y, 0, sizeof(Connected_y));
  memset(Connected_light, 0, sizeof(Connected_light));
  areamax = 0;
  labelcount = 0;
  labelmax = 1;
  int flag_1;
  int last_labelIndex = 0;

  for (i = 1; i <= MT9V03X_H - 1; i++)
  {
    for (j = 1; j <= MT9V03X_W - 1; j++)
    {
      index = i * (MT9V03X_W) + j;

      if (bitmap[index] == 1 && labelmap[index] == 0)
      {
        if (labelIndex >= 255) { i = MT9V03X_H; break; }  // 防止数组越界
        labelIndex++;
        Connected_xmin[labelIndex] = 32767;
        Connected_ymin[labelIndex] = 32767;
        Connected_xmax[labelIndex] = 0;
        Connected_ymax[labelIndex] = 0;
        SearchNeighbor_4(bitmap, labelmap, labelIndex, index, &queue, i, j, (unsigned char *)mt9v03x_image);
        popIndex = PopQueue(&queue);
        while (popIndex > -1)
        {
          SearchNeighbor_4(bitmap, labelmap, labelIndex, popIndex, &queue, i, j, (unsigned char *)mt9v03x_image);
          popIndex = PopQueue(&queue);
        }
      }
      flag_1 = labelIndex - last_labelIndex;
      last_labelIndex = labelIndex;

      if (flag_1 == 1)
      {
        area_map[labelIndex] = area;

        Connected_x[labelIndex] = (int)(Connected_x[labelIndex] / area);
        Connected_y[labelIndex] = (int)(Connected_y[labelIndex] / area);
        //Connected_light[labelIndex] = (int)(Connected_light[labelIndex] / area);

        flag_1 = 0;
      }

      if (x != labelIndex)
      {
        x = labelIndex;
        if (area > areamax)
        {
          areamax = area;
          labelmax = labelIndex;
        }

        if (area <= 5)
        {
          labeldel[del] = labelIndex;
          del++;
          if (del >= 9) del = 9;
        }

        area = 0;
      }
    }
  }
  // 不需要 free(queue) — 栈上分配，自动回收

  labelcount = labelIndex - 1;
}

/********************************************
 *  信标权重判断（改进版）
 *  新增：面积上限过滤 + 圆形置信度硬性过滤 + 面积惩罚项
 *  面积范围：15~750像素（对应直径5~30像素的实心圆）
 *  圆形检测得分：必须 > 0.4 才可能是信标
 ********************************************/
int quanzhi_weight[256];
void quanzhong_light_panduan(void)
{
    float circle_score;
    uint16_t area_val;
    memset(quanzhi_weight, 0, sizeof(quanzhi_weight));
    for (int i = 2; i <= labelcount + 1; i++)
    {
        area_val = area_map[i];

        // === 第1关：面积硬性过滤（5~30像素直径圆形信标）===
        // 最小面积: π*(2.5)^2 ≈ 20, 取15留余量
        // 最大面积: π*(15)^2 ≈ 707, 取750留余量
        if (area_val < 15 || area_val > 750)
        {
            quanzhi_weight[i] = 0;
            continue;
        }

        // === 第2关：圆形检测（8方向+变异系数+宽高比+填充率）===
        circle_score = Square_detection(Connected_x[i], Connected_y[i]);
        if (circle_score < 0.4f)  // 不够圆，过滤大片不规则光斑
        {
            quanzhi_weight[i] = 0;
            continue;
        }

        // === 第3关：空间位置过滤（角落/边缘排除）===
        if (((Connected_x[i] < 30 && (Connected_y[i] < 30 || Connected_y[i] > 100))
             || (Connected_x[i] > 130 && (Connected_y[i] < 30 || Connected_y[i] > 100)))
            && area_val > 10)
        {
            quanzhi_weight[i] = 0;
            continue;
        }
        if ((Connected_x[i] > 54 && Connected_x[i] < 74
             && Connected_y[i] > 30 && Connected_y[i] < 100)
            && area_val < 10)
        {
            quanzhi_weight[i] = 0;
            continue;
        }

        // === 第4关：加权打分（改进版）===
        // 位置分: 越靠近图像中心(94,64)得分越高
        float pos_score = (1.0f - fabsf((float)Connected_x[i] - 94.0f) / 94.0f)
                        + (1.0f - fabsf((float)Connected_y[i] - 64.0f) / 64.0f);

        // ★ 面积分（关键修改）：用高斯型权重，最优面积≈100（直径~11px中等距离信标）
        //   面积偏离最优值越远分越低，不再奖励大面积
        float area_opt = 100.0f;  // 最优面积（可根据实际信标大小调整）
        float area_dev = fabsf((float)area_val - area_opt) / area_opt;
        float area_score = (area_dev < 1.0f) ? (1.0f - area_dev) : 0.0f;

        // 亮度分: 使用平均亮度而非累计亮度，避免大面积光斑占优
        float avg_brightness = (float)Connected_light[i] / (float)area_val;

        // 综合权重：位置(40%) + 圆形度(30%) + 面积适中(15%) + 平均亮度(15%)
        quanzhi_weight[i] = (int)(pos_score * 40000.0f
                                + circle_score * 30000.0f
                                + area_score * 15000.0f
                                + avg_brightness * 15.0f);
    }
}
int center_x ; 
int center_y ;
//Ѱ�����Ȩ��ֵ
int find_max(void)
{
    int wei=2;
    int maxx=quanzhi_weight[2];
    for(int i=3;i<=labelcount+1;i++)
    {
        if(quanzhi_weight[i]>maxx)
        {
            maxx = quanzhi_weight[i];
            wei=i;
            why = wei;
        }
    }
     //����
          
          float screen_x = Connected_x[wei] * 128.0f / MT9V03X_W;
          float screen_y = Connected_y[wei] * 81.0f / MT9V03X_H;
          
          center_x =Connected_x[wei]; 
          center_y =Connected_y[wei];
          //xx = (int)screen_x;
          //yy = (int)screen_y;
          
           //ӳ�䵽Ŀ��ֱ��� (128��81)
          //float screen_x = center_x[wei] * (128.0f / MT9V03X_W);
          //float screen_y = center_y[wei] * (81.0f / MT9V03X_H);
          //float screen_x = center_x[wei] * (127.0f / (MT9V03X_W - 1));  // ӳ�䵽0-127
          //float screen_y = center_y[wei] * (81.0f / (MT9V03X_H - 1));   // ӳ�䵽0-81
          //xx = (int)screen_x;
          //yy = (int)screen_y;
           xx = (int)screen_x;  // ��������
           yy = (int)screen_y;
          
          //xx=(int)center_x[wei];
          //yy=(int)center_y[wei];
          
          if(xx>=127)xx=127;
          if(xx<=0)xx=0;
          if(yy>=81)yy=81;
          if(yy<=0)yy=0;
    return wei;
}

float KKKK;




//�������Ȩ��Ŀ��λ��
/*void quanzhong_light(void)
{
    dotcnt=0;
    int zxc=find_max();
    int16_t comparative=MT9V03X_W/2-sumlie;
                 //�̱�   ����
    

    if_look_light = 0;

    if(quanzhi_weight[zxc]!=0)
    {
        QQQQ = 1;//����ת��ֵ��־

        if_look_light = 1;//��������

        keep_going = 70;//�ֶ���ʱ
        eye_look_light = 1;
        sumlie = Connected_y[zxc];
        sumhang = Connected_x[zxc];
        dotcnt++;
        //move_k1 = 0.015;

        target_turn = comparative ;
if(sumhang >=80 )
{

    target_turn = 0 ;

}



        if (count19 ==1 )
            {
                count20 = 20 ;
                count19 = 0 ;
            }
            else
            {
                count20 = 0 ;
            }

        if(target_turn <= 7 + count20 && target_turn >= -7 - count20 )
        {

            target_turn = 0;
            count19 = 1;

        }

        ips200_show_int(150,130,target_turn,8);
        if(target_turn  <= 25 && target_turn  >= -25 )//&& double_speed() <= 700)
            {


              target_turn = 0;
              count21 ++ ;
              if(count21 >= 80)
              {

              set_speed = 0;

              }


            }
            else
            {
                count21 = 0;
                set_speed = 0;
            }

    }

    else
    {
      //  move_k1 = 0.015;
        keep_going-- ;
        if(keep_going <= 10)//��ʱ����ֹͣ
        {
           // qianhou_zhongzhi = 1.54;
            set_speed = 0;
            keep_going = 5;
            if_turn = 1 ;
        }
        else if(keep_going > 10)//��ʱ���ڼ���ǰ��
        {
            //qianhou_zhongzhi = 3.0;
            set_speed = 0 ;
            if_turn = 0 ;
        }

        ips200_show_int(150,130,target_turn,8);

        if (QQQQ == 1)  //��ֹ���ж�����ֵ����������ֻ����һ��,1,һ��ʼ�Ϳ����ƣ�Ȼ�󿴲�������1��0
        {               //2,һ��ʼ���������ƣ�Ȼ�󿴼�����0��1
        target_turn = 0 ;
        QQQQ = 0 ;
        }
        eye_look_light = 2;//��ʾ��������
    }
}
*/

/*---------------------------------------------------------------
����    ����Bin_Image_Filter
����    �ܡ�������㣬���һ����ɫ���ر��϶�İ�ɫ���ذ�Χ�������������������Ϊ��ɫ���������Ϊ�����������ĺ�ɫ�����㡣
         ���һ����ɫ���ر����ٵİ�ɫ���ذ�Χ�������������������Ϊ��ɫ���������Ϊ�����������İ�ɫ�����㡣
����    ������
���� �� ֵ����
��ע�����
----------------------------------------------------------------*/
void Bin_Image_Filter (void)
{
  int  nr; //��
  int  nc; //��

  for (nr = 1; nr < MT9V03X_H - 1; nr++)
  {
    for (nc = 1; nc < MT9V03X_W- 1; nc = nc + 1)
    {
      if ((mt9v03x_image_BandW[nr][nc] == 0)
          && (mt9v03x_image_BandW[nr - 1][nc] + mt9v03x_image_BandW[nr + 1][nc] + mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] > 2))
      {
          mt9v03x_image_BandW[nr][nc] = 1;
      }
      else if ((mt9v03x_image_BandW[nr][nc] == 1)
               && (mt9v03x_image_BandW[nr - 1][nc] + mt9v03x_image_BandW[nr + 1][nc] + mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] < 2))
      {
          mt9v03x_image_BandW[nr][nc] = 0;
      }
    }
  }
}

/*************************************************************************
*  �������ƣ�void eye_show(void)
*  ����˵�������ӻ�ʶ������ű�Ƶ�λ�ã�����ʾ��ʮ��׼��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2022��3��31��
*  ��    ע��  Bin_Image[LCDH][LCDW]; ��ֵ����ͼ������
*************************************************************************/
void eye_show (void)//��ʾʮ��
{
  for(int i = 1 ;i<MT9V03X_H-1;i++)
  {
      mt9v03x_image_BandW[i][sumlie] = 1;
  }
  for(int i = 1 ;i<MT9V03X_W-1;i++)
  {
      mt9v03x_image_BandW[sumhang][i] = 1;
  }
    return;
}

/**************************************************************************
*                                                                          *
*  �������ƣ�Seek_Road(void)                                                   *
*  ����˵����Ѱ�Ұ�ɫ����ƫ��ֵ                                                      *
*  ����˵������                                                            *
*  �������أ�ֵ�Ĵ�С                                                      *
*  �޸�ʱ�䣺2017-07-16                                                    *
*  ��    ע�����м�Ϊ0������һ���Ҳ��һ����ֵ����1�����                            *
*            ��������ӵ�һ�п�ʼ�������ڶ��н�����                                 *
*            ������Ϊ��������ֵԽ��˵��Խƫ��ߣ�                                  *
*            ������Ϊ��������ֵԽ��˵��Խƫ�ұߡ�                                  *
**************************************************************************/
void Seek_Road (void)
{
    int16_t nr; //��
    int16_t nc; //��
    int16_t temp = 0; //��ʱ��ֵ
    //for(nr=1; nr<MAX_ROW-1; nr++)
    temp = 0;
    int16 MAX_COL=MT9V03X_W;   //�Լ�д�ģ��п����д�
    for (nr = 8; nr < 40; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET0 = temp;
    temp = 0;
    for (nr = 40; nr < 80; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET1 = temp;
    temp = 0;
    for (nr = 80; nr < 110; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET2 = temp;
    return;
}

/*****************************************************************************
*函数名称：void PushQueue(Queue *queue, int data)
*功能说明：入队（固定大小循环数组，O(1)，无 malloc）
*参数说明：queue: 队列指针  data: 入队数据
*返回值：  无
*修改时间：2026年7月7日
*备    注：队列满时静默丢弃，避免堆耗尽导致卡死
******************************************************************************/
void PushQueue(Queue *queue, int data)
{
    if (queue->count >= QUEUE_MAX_SIZE) {
        return;  // 队列满，丢弃（嵌入式安全策略）
    }
    queue->data[queue->tail] = data;
    queue->tail = (queue->tail + 1) % QUEUE_MAX_SIZE;
    queue->count++;
}
//
/*****************************************************************************
*函数名称：int PopQueue(Queue *queue)
*功能说明：出队（固定大小循环数组，O(1)，无 free）
*参数说明：queue: 队列指针
*返回值：  队首元素值，队列空时返回 -1
*修改时间：2026年7月7日
*备    注：无动态内存，不会因 free 失败导致卡死
******************************************************************************/
int PopQueue(Queue *queue)
{
    if (queue->count == 0) {
        return -1;  // 队列空
    }
    int data = queue->data[queue->head];
    queue->head = (queue->head + 1) % QUEUE_MAX_SIZE;
    queue->count--;
    return data;
}


/*************************************************************************
*  函数名称：float Square_detection (int x,int y)
*  功能说明：圆形检测（8方向半径一致性+变异系数+宽高比+尺寸约束）
*           专为5~30像素直径实心圆形信标灯设计，过滤不规则大光斑
*  参数说明：x,y — 连通域质心坐标
*  返回值：  圆形置信度 0.0~1.0，>0.4 认为可能是圆形信标
*  修改时间：2022年7月11日（原始）/ 2026年7月16日（重写）
*  备    注：  mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W]; 二值化图像数据
*************************************************************************/
// 8方向偏移表：上、右上、右、右下、下、左下、左、左上
static const int8_t s8_dir_dx[8] = { 0,  1,  1,  1,  0, -1, -1, -1};
static const int8_t s8_dir_dy[8] = {-1, -1,  0,  1,  1,  1,  0, -1};

float Square_detection(int16_t x, int16_t y)
{
    int16_t nr = y; // 行（质心Y）
    int16_t nc = x; // 列（质心X）
    int16_t radii[8];
    int16_t i, r, cx, cy;
    int16_t r_min = 127, r_max = 0, r_sum = 0;
    int16_t valid_dirs = 0;
    float r_avg, variance, cv;

    // 边界保护：距图像边缘至少6像素，防止越界
    if (nr < 6 || nr > MT9V03X_H - 7 || nc < 6 || nc > MT9V03X_W - 7)
        return 0.0f;

    // 质心必须是白点（信标像素）
    if (mt9v03x_image_BandW[nr][nc] != 1)
        return 0.0f;

    // === 第1步：8方向半径测量 ===
    for (i = 0; i < 8; i++)
    {
        r = 0;
        cx = nc;
        cy = nr;
        while (1)
        {
            cx += s8_dir_dx[i];
            cy += s8_dir_dy[i];
            // 越界检查
            if (cx < 0 || cx >= MT9V03X_W || cy < 0 || cy >= MT9V03X_H)
                break;
            if (mt9v03x_image_BandW[cy][cx] != 1)
                break;
            r++;
            if (r > 18) break;  // 安全上限：直径>36像素不可能
        }
        radii[i] = r;
        if (r > 0) valid_dirs++;
    }

    // 至少6个方向有有效半径（排除线状或噪声）
    if (valid_dirs < 6) return 0.0f;

    // === 第2步：半径统计（最小/最大/平均）===
    for (i = 0; i < 8; i++)
    {
        if (radii[i] > 0)
        {
            r_sum += radii[i];
            if (radii[i] < r_min) r_min = radii[i];
            if (radii[i] > r_max) r_max = radii[i];
        }
    }
    r_avg = r_sum / (float)valid_dirs;

    // === 第3步：尺寸约束（直径5~30像素 → 半径2.5~15像素）===
    if (r_avg < 2.0f || r_avg > 15.0f) return 0.0f;

    // 极端半径比保护：最大半径不超过最小半径的2.5倍
    // 大面积不规则光斑在此被过滤
    if (r_min > 0 && (float)r_max / (float)r_min > 2.5f) return 0.0f;

    // === 第4步：半径变异系数 CV = σ/μ ===
    // 完美圆 CV→0，不规则形状 CV>0.35
    variance = 0.0f;
    for (i = 0; i < 8; i++)
    {
        if (radii[i] > 0)
        {
            float diff = radii[i] - r_avg;
            variance += diff * diff;
        }
    }
    variance /= (float)valid_dirs;
    cv = sqrtf(variance) / r_avg;  // 变异系数

    if (cv > 0.35f) return 0.0f;  // 太不圆，不规则光斑在此被过滤

    // === 第5步：宽高比（从水平和垂直方向半径计算）===
    // 左(6) + 右(2) = 水平方向，上(0) + 下(4) = 垂直方向
    float half_w = (radii[6] + radii[2]) / 2.0f + 1.0f;
    float half_h = (radii[0] + radii[4]) / 2.0f + 1.0f;
    float aspect = half_w / half_h;
    if (aspect < 0.55f || aspect > 1.8f) return 0.0f;

    // === 第6步：综合打分 ===
    // CV越小越圆，宽高比越接近1.0越圆
    float score = 1.0f - cv * 2.0f - fabsf(aspect - 1.0f) * 0.4f;
    if (score < 0.0f) score = 0.0f;
    if (score > 1.0f) score = 1.0f;

    return score;
}

