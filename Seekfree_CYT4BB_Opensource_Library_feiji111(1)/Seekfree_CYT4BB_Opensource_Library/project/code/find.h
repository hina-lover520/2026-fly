/*
 * find.h
 *
 *  Created on: 2025��4��26��
 *      Author: 39653
 */
#include "zf_common_headfile.h"
#ifndef CODE_FIND_H_
#define CODE_FIND_H
extern float screen_x;
extern float screen_y;
extern int xx;
extern int yy;
extern int center_x;  // ʹ��0.5f�����ǳ���2
extern int center_y;
extern uint16_t areamax;

extern short Connected_xmin[256];//最小行值
extern short Connected_ymin[256];//最小列值
extern short Connected_xmax[256];//最大行数
extern short Connected_ymax[256];//最大列数

extern uint8_t labelBuffer[MT9V03X_W*MT9V03X_H];
extern uint8_t labelcount;
extern int why;
void Seek_Beacon (void);
void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char * labelmap);
extern uint8 mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W];
extern uint8 display_image[MT9V03X_H][MT9V03X_W];
void Set_image_twovalues(uint8 value);  //���ö�ֵ��
void Set_image_two(void);
void Bin_Image_Filter (void);//�������
// 固定大小循环数组队列，避免 malloc 耗尽嵌入式堆空间
#define QUEUE_MAX_SIZE 4096
typedef struct Queue
{
    int data[QUEUE_MAX_SIZE];
    int head;
    int tail;
    int count;
} Queue;
void PushQueue(Queue *queue, int data);
int PopQueue(Queue *queue);
extern int16_t OFFSET0;    // ��Զ������������ֵ�ۺ�ƫ����
extern int16_t OFFSET1;    // �ڶ���
extern int16_t OFFSET2;    // �����������
void eye_show (void);
float Square_detection (int16_t x,int16_t y);
#endif /* CODE_FIND_H_ */
