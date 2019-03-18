#include <stdio.h>
#include <graphics.h>
#include <stdlib.h>
#include <math.h>
#include <conio.h>
#include <time.h>

#define PI 3.1415926
#define SCREEN_LENGTH	1920
#define SCREEN_WIDTH	1080

//�����漰�����ȵĵ�λ��Ϊ���أ�ÿ���ش���ʵ�ʳ���0.45��
#define MAXNUM	100000	//�������Ŀ
#define MAXTIME	100001
#define MAX_A	0.7	//�����ٶȣ��൱��4.4m/s2.
#define MIN_A	-1	//��С���ٶȣ���ɲ�����ٶȣ��൱��-4.4m/s2.
#define MAX_V	5	//����1-5�����ֱ��൱��(16.2km/h, 32.4km/s, 48.6km/s, 64.8km/h, 81km/h)
#define MIN_V	0	//���ܵ���
#define VEH_LENGTH	10	//������Լ10*0.45=4.5��
#define VEH_WIDTH	4	//����Լ4*0.45=1.8��
#define MIN_DIS 3	//��С�����
#define MAX_DIS 20	//��󳵾ࣨ��������ڴ�ֵʱ���������٣�ֱ���ﵽ����ٶȻ�ﵽ��С�����
#define FREQU	100	//ˢ��Ƶ��Ϊ100ms
#define LANE_WIDTH	20	//������ȣ�Լ16*0.45=7.2�ף���ʵ�ʳ�������Ϊ�˷Ŵ󽻲���Ա������
#define ARC_RAD		16	//�����Բ�ǻ��Ȱ뾶
#define CROSS_X	(SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD)
#define CROSS_Y	(SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD)
#define CROSS_WIDTH	(3*LANE_WIDTH)
#define CROSS_LENGTH	(3*LANE_WIDTH)

#define LEFT_TURN_DIS	(PI/2*(ARC_RAD + 7*LANE_WIDTH/2))	//��ת����ת�����
#define RIGHT_TURN_DIS	(PI/2*(ARC_RAD + LANE_WIDTH/2))		//��ת����ת�����
#define HAVE_SIGNAL	true
#define RAD_LIGHT	5	//�źŵư뾶

//����ͣ���ߵ�X(Y)����
#define LEFT_STOP_LINE		SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD
#define RIGHT_STOP_LINE		SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD
#define TOP_STOP_LINE		SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD
#define BOTTOM_STOP_LINE	SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD

//���̵�ʱ��
#define TIME_EAST	200	//���ö��������̵�ʱ��Ϊ20��
#define TIME_SOUTH	200	//�����ϱ������̵�ʱ��Ϊ20��
#define TIME_YELLOW	50	//���ʱ�䣬���Ƶ�ʱ��

typedef struct
{
	int x;
	int y;
} point;	//�����

int time_total = 0;	//��ǰʱ�䲽
int num = 0;	//������Ŀ
int cur_temp[12];	//��ʱ���ÿ�����ĵ�ǰ�����
COLORREF color[6] = {GREEN, LIGHTRED, YELLOW, LIGHTBLUE, CYAN, RED};

bool signal[4] = {true, false, true, false};	//�ĸ����̵ƣ�true��ʾ�̵ƣ�false��ʾ���
point red_light[4] = {{LEFT_STOP_LINE+RAD_LIGHT, SCREEN_WIDTH/2+RAD_LIGHT},
{SCREEN_LENGTH/2+RAD_LIGHT, BOTTOM_STOP_LINE-RAD_LIGHT},
{RIGHT_STOP_LINE-RAD_LIGHT, SCREEN_WIDTH/2-RAD_LIGHT},
{SCREEN_LENGTH/2-RAD_LIGHT,TOP_STOP_LINE+RAD_LIGHT}};
point green_light[4] = {{LEFT_STOP_LINE+RAD_LIGHT, SCREEN_WIDTH/2+3*RAD_LIGHT},
{SCREEN_LENGTH/2+3*RAD_LIGHT, BOTTOM_STOP_LINE-RAD_LIGHT},
{RIGHT_STOP_LINE-RAD_LIGHT, SCREEN_WIDTH/2-3*RAD_LIGHT},
{SCREEN_LENGTH/2-3*RAD_LIGHT,TOP_STOP_LINE+RAD_LIGHT}};


//bool cross_occ[CROSS_LENGTH][CROSS_WIDTH];	//��־������ڸ�����λ���Ƿ�ռ��

typedef struct
{
	int x;		//x���꣬С�����ĵ��X����
	int y;		//y���꣬С�����ĵ��Y����
	double v;	//�ٶȣ�Ϊ����
	double a;	//���ٶȣ�Ϊ������ʹ���������ʻ��
	COLORREF c;	//��ɫ
	int lane;	//�������
	int pre_veh;//��ǰ��������һ����������
	int flag;	//��ǳ���״̬��0Ϊ��Ч��1Ϊ��Ч����Ч�������Ѿ�ʻ��������ĳ���
	double ang;	//ǰ����ʻ�Ƕȣ�����Ϊ0�ȣ�˳ʱ��������2PI.
	int turn_time;
	bool arrive_flag;	//���뽻��ڱ�־�������Ϊ1
	bool leave_flag;	//������ڱ�־����ȥ��Ϊ1
	double omega;		//ת����ٶ�
} vehicle;

vehicle veh[MAXNUM];	//���泵����Ϣ

//������������
int lane_center[12] = {SCREEN_WIDTH/2+LANE_WIDTH/2, SCREEN_WIDTH/2+3*LANE_WIDTH/2, SCREEN_WIDTH/2+5*LANE_WIDTH/2,
					SCREEN_LENGTH/2+LANE_WIDTH/2,SCREEN_LENGTH/2+3*LANE_WIDTH/2,SCREEN_LENGTH/2+5*LANE_WIDTH/2, 
					SCREEN_WIDTH/2-LANE_WIDTH/2,SCREEN_WIDTH/2-3*LANE_WIDTH/2,SCREEN_WIDTH/2-5*LANE_WIDTH/2,
					SCREEN_LENGTH/2-LANE_WIDTH/2,SCREEN_LENGTH/2-3*LANE_WIDTH/2,SCREEN_LENGTH/2-5*LANE_WIDTH/2,};

//������ת��Բ������,����Ϊ0,0��ʾֱ�г���
point turn_circle[12] = \
{
	{LEFT_STOP_LINE,TOP_STOP_LINE},{0,0},{LEFT_STOP_LINE,BOTTOM_STOP_LINE},
	{LEFT_STOP_LINE,BOTTOM_STOP_LINE},{0,0},{RIGHT_STOP_LINE, BOTTOM_STOP_LINE},
	{RIGHT_STOP_LINE, BOTTOM_STOP_LINE},{0,0},{RIGHT_STOP_LINE, TOP_STOP_LINE},
	{RIGHT_STOP_LINE, TOP_STOP_LINE},{0,0},{LEFT_STOP_LINE,TOP_STOP_LINE},
};

//������ת��뾶��0��ʾֱ�г���
int turn_rad[12] = \
{ARC_RAD+7*LANE_WIDTH/2,0,ARC_RAD+LANE_WIDTH/2,ARC_RAD+7*LANE_WIDTH/2,0,ARC_RAD+LANE_WIDTH/2,ARC_RAD+7*LANE_WIDTH/2,0,ARC_RAD+LANE_WIDTH/2,ARC_RAD+7*LANE_WIDTH/2,0,ARC_RAD+LANE_WIDTH/2};
/*12�������ĳ�����ʼλ��*/
point init_pos[12] = \
{
	{0, lane_center[0]},	//����0
	{0, lane_center[1]}, //����1
	{0, SCREEN_WIDTH/2+5*LANE_WIDTH/2},//����2

	{SCREEN_LENGTH/2+LANE_WIDTH/2, SCREEN_WIDTH}, //����3
	{SCREEN_LENGTH/2+3*LANE_WIDTH/2, SCREEN_WIDTH}, //����4
	{SCREEN_LENGTH/2+5*LANE_WIDTH/2, SCREEN_WIDTH},//����5

	{SCREEN_LENGTH, SCREEN_WIDTH/2-LANE_WIDTH/2}, //����6
	{SCREEN_LENGTH, SCREEN_WIDTH/2-3*LANE_WIDTH/2},
	{SCREEN_LENGTH, SCREEN_WIDTH/2-5*LANE_WIDTH/2},//����8

	{SCREEN_LENGTH/2-LANE_WIDTH/2, 0}, //����9
	{SCREEN_LENGTH/2-3*LANE_WIDTH/2, 0}, //����10
	{SCREEN_LENGTH/2-5*LANE_WIDTH/2, 0},//����11
};

//����signal�����ֵ�����ú��̵ơ�signal[0],[1],[2],[3]�ֱ��ʾ���������ĸ�����ĵơ�
void set_light()
{
	int t = time_total % 500;
	if(t==0)
	{
		signal[0] = signal[2] = true;
		signal[1] = signal[3] = false;
	}
	else if(t==200)
	{
		signal[0] = signal[2] = false;
		signal[1] = signal[3] = false;
	}
	else if(t==250)
	{
		signal[0] = signal[2] = false;
		signal[1] = signal[3] = true;
	}
	else if(t==450)
	{
		signal[0] = signal[2] = false;
		signal[1] = signal[3] = false;
	}
	else return;
	for(int i=0;i<4;i++)
	{
		if(signal[i])
		{
			//���õ���
			clearcircle(red_light[i].x, red_light[i].y, RAD_LIGHT);
			clearcircle(green_light[i].x, green_light[i].y, RAD_LIGHT);

			setfillcolor(GREEN);	//�����̵���
			solidcircle(green_light[i].x, green_light[i].y, RAD_LIGHT);
		}
		else
		{
			//���õ���
			clearcircle(green_light[i].x, green_light[i].y, RAD_LIGHT);
			clearcircle(red_light[i].x, red_light[i].y, RAD_LIGHT);

			setfillcolor(RED);	//���ú����
			solidcircle(red_light[i].x, red_light[i].y, RAD_LIGHT);
		}
	}
}
void init_intersection()
{
	setbkcolor(DARKGRAY);	//���ñ�����ɫ
	cleardevice();
	setlinestyle(PS_SOLID);
	//������
	setlinecolor(YELLOW);
	line(0, SCREEN_WIDTH/2, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2);	//��1
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2, SCREEN_LENGTH, SCREEN_WIDTH/2);//��2
	line(SCREEN_LENGTH/2, 0, SCREEN_LENGTH/2, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	//��1
	line(SCREEN_LENGTH/2, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2, SCREEN_WIDTH);//��2
	setlinecolor(WHITE);

	//������
	line(0, SCREEN_WIDTH/2-3*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH);
	line(0, SCREEN_WIDTH/2+3*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2-3*LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2+3*LANE_WIDTH);

	//������
	line(SCREEN_LENGTH/2-3*LANE_WIDTH, 0, SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH, 0, SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	

	line(SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH);	//��1
	line(SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH);	//��1

	//��������ཻ����������Ϊ˳ʱ��1,2,3,4
	arc(SCREEN_LENGTH/2-3*LANE_WIDTH-2*ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH-2*ARC_RAD, SCREEN_LENGTH/2-3*LANE_WIDTH,SCREEN_WIDTH/2-3*LANE_WIDTH, 1.5*PI, 2*PI);
	arc(SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-2*ARC_RAD, SCREEN_LENGTH/2+3*LANE_WIDTH+2*ARC_RAD,SCREEN_WIDTH/2-3*LANE_WIDTH, PI, 1.5*PI);
	arc(SCREEN_LENGTH/2-3*LANE_WIDTH-2*ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH,SCREEN_WIDTH/2+3*LANE_WIDTH+2*ARC_RAD, 0, 0.5*PI);
	arc(SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH, SCREEN_LENGTH/2+3*LANE_WIDTH+2*ARC_RAD,SCREEN_WIDTH/2+3*LANE_WIDTH+2*ARC_RAD, 0.5*PI, PI);

	//��ͣ���� �ֱ�Ϊ ���ң��ϣ���
	line(SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH, SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH);

	line(SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);
	line(SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD);

	//������
	setlinestyle(PS_DASH);
	//������
	line(0, SCREEN_WIDTH/2-2*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2-2*LANE_WIDTH);
	line(0, SCREEN_WIDTH/2-LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2-LANE_WIDTH);
	line(0, SCREEN_WIDTH/2+LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2+LANE_WIDTH);
	line(0, SCREEN_WIDTH/2+2*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2+2*LANE_WIDTH);
	//������
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2-2*LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2-2*LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2-LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2-LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2+LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2+LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2+2*LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2+2*LANE_WIDTH);
	//������
	line(SCREEN_LENGTH/2-LANE_WIDTH, 0, SCREEN_LENGTH/2-LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	
	line(SCREEN_LENGTH/2-2*LANE_WIDTH, 0, SCREEN_LENGTH/2-2*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	
	line(SCREEN_LENGTH/2+LANE_WIDTH, 0, SCREEN_LENGTH/2+LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	
	line(SCREEN_LENGTH/2+2*LANE_WIDTH, 0, SCREEN_LENGTH/2+2*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	
	//������	
	line(SCREEN_LENGTH/2-LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2-LANE_WIDTH, SCREEN_WIDTH);//��2
	line(SCREEN_LENGTH/2-2*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2-2*LANE_WIDTH, SCREEN_WIDTH);//��2
	line(SCREEN_LENGTH/2+LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2+LANE_WIDTH, SCREEN_WIDTH);//��2
	line(SCREEN_LENGTH/2+2*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2+2*LANE_WIDTH, SCREEN_WIDTH);//��2

	//���źŵ����
	rectangle(LEFT_STOP_LINE,SCREEN_WIDTH/2,LEFT_STOP_LINE+2*RAD_LIGHT,SCREEN_WIDTH/2+4*RAD_LIGHT);
	rectangle(SCREEN_LENGTH/2, BOTTOM_STOP_LINE-2*RAD_LIGHT,SCREEN_LENGTH/2+4*RAD_LIGHT,BOTTOM_STOP_LINE);
	rectangle(RIGHT_STOP_LINE-2*RAD_LIGHT,SCREEN_WIDTH/2-4*RAD_LIGHT,RIGHT_STOP_LINE,SCREEN_WIDTH/2);
	rectangle(SCREEN_LENGTH/2-4*RAD_LIGHT,TOP_STOP_LINE,SCREEN_LENGTH/2,TOP_STOP_LINE+2*RAD_LIGHT);

	setlinestyle(PS_SOLID);
}

/*��С����С�������Ѿ�����ʼ��*/
void vehicle_draw(int i)
{
	if(!veh[i].flag)
		return;
	//���û�������
	int left, top, right, bottom, lane = veh[i].lane;
	if(lane==0 || lane==1 || lane==2 || lane==6 || lane==7 || lane==8)
	{
		left = veh[i].x - VEH_LENGTH/2;
		right = left + VEH_LENGTH;
		top = veh[i].y - VEH_WIDTH/2;
		bottom = top + VEH_WIDTH;
	}
	else
	{
		left = veh[i].x - VEH_WIDTH/2;
		right = left + VEH_WIDTH;
		top = veh[i].y - VEH_LENGTH/2;
		bottom = top + VEH_LENGTH;
	}
	if(left<0 || top<0 || right>SCREEN_LENGTH || bottom>SCREEN_WIDTH)
		return;
	setfillcolor(veh[i].c);
	setfillstyle(BS_SOLID);
	setlinecolor(veh[i].c);
	setlinestyle(BS_SOLID);
	setpolyfillmode(ALTERNATE);
	if(veh[i].turn_time==0)	//δת�䳵��
	{
		solidrectangle(left, top, right, bottom);
//		RECT r = {10, 10, 50, 50};
//		char s[5];
//		settextcolor(WHITE);
//		outtextxy(veh[i].x, veh[i].y, itoa(i,s,10));
	}
	else	//����ת��ĳ���
	{
		double ang = -1 * (veh[i].ang - ((int)(lane/3)) * (PI / 2));		
		int x0[4] = {left, right, left, right};
		int y0[4] = {top, top, bottom, bottom};
		int x1[4], y1[4];
		int cenx = veh[i].x, ceny = veh[i].y;
		
		for(int i=0;i<4;i++)
		{
			x1[i] = (int)((x0[i] - cenx) * cos(ang) - (y0[i] - ceny) * sin(ang) + cenx);
			y1[i] = (int)((x0[i] - cenx) * sin(ang) + (y0[i] - ceny) * cos(ang) + ceny);	
		}
		POINT pts[] = {{x1[0],y1[0]},{x1[2],y1[2]},{x1[3],y1[3]},{x1[1],y1[1]}};
		fillpolygon((POINT*)pts, 4);
	}	
}

/*��������*/
void create_vehicle(int index)
{
	int lane;
	srand((unsigned)time(NULL));
	if(index >= MAXNUM)
	{
		printf("������Ŀ����!\n");
		exit(1);
	}
	//��ʼ��С��
	lane = rand() % 12;	//�������
	veh[index].c = color[rand()%6];
	veh[index].lane = lane;
	veh[index].x = init_pos[lane].x;
	veh[index].y = init_pos[lane].y;

	veh[index].v = (3+rand()%3);	//���ó�ʼ�ٶȴ�С
	veh[index].a = 0;				//���ó�ʼ���ٶȴ�С

	veh[index].pre_veh = cur_temp[lane];	//��¼ǰ������
	if(veh[index].pre_veh==0)
		veh[index].v = MAX_V;
	veh[index].flag = 1;
	veh[index].ang = ((int)(lane/3)) * (PI / 2);
	cur_temp[lane] = index;		//���õ�ǰ������ǰ��������
	veh[index].turn_time = 0;
	//����С��
	vehicle_draw(index);
}

void refresh_vehicles()
{
	int i;
	setwritemode(R2_XORPEN);	//���ģʽ���ڶ��λ��������ڲ���
	for(i=1;i<=num;i++)		//����
	{
		if(veh[i].flag == 0) continue;	//������Ч��
		vehicle_draw(i);
	}
	Sleep(FREQU);
	for(i=1;i<=num;i++)		//����
	{
		if(veh[i].flag == 0) continue;	//������Ч��
		vehicle_draw(i);
	}
}

/*���¼��ٶ�*/
void update_a()
{
	for(int i=1;i<=num;i++)
	{
/*

		int lane = veh[i].lane;
		switch(lane)
		{
		case 0:
		case 1:
		case 2:
			if(veh[i].x+VEH_LENGTH/2 >= LEFT_STOP_LINE && veh[i].arrive_flag==0)
			{
				veh[i].arrive_flag = 1;
				res = true;
			}
			break;
		case 3:	
		case 4:
		case 5:
			if(veh[i].y-VEH_LENGTH/2 <= BOTTOM_STOP_LINE && veh[i].arrive_flag==0)
			{
				veh[i].arrive_flag = 1;
				res = true;
			}
			break;
		case 6:
		case 7:
		case 8:
			if(veh[i].x-VEH_LENGTH/2 <= RIGHT_STOP_LINE && veh[i].arrive_flag==0)
			{
				veh[i].arrive_flag = 1;
				res = true;
			}
			break;
		case 9:
		case 10:
		case 11:
			if(veh[i].y+VEH_LENGTH/2 >= TOP_STOP_LINE && veh[i].arrive_flag==0)
			{
				veh[i].arrive_flag = 1;
				res = true;
			}
			break;
		default:
			res = false;

*/


		if(veh[i].flag == 0) continue;	//������Ч��
		if(veh[i].v==MAX_V||veh[i].v==0)
			veh[i].a = 0;
		else
		{	
			double dif_v = veh[veh[i].pre_veh].v - veh[i].v;
			veh[i].a = dif_v < MAX_A ? dif_v : MAX_A;
		}
	}
}

/*�жϵ��ｻ��ڵ�һ˲��*/
bool arrive_intersection(int i)
{
	int lane = veh[i].lane;
	bool res = false;
	switch(lane)
	{
	case 0:
	case 1:
	case 2:
		if(veh[i].x+VEH_LENGTH/2 >= LEFT_STOP_LINE && veh[i].arrive_flag==0)
		{
			veh[i].arrive_flag = 1;
			res = true;
		}
		break;
	case 3:	
	case 4:
	case 5:
		if(veh[i].y-VEH_LENGTH/2 <= BOTTOM_STOP_LINE && veh[i].arrive_flag==0)
		{
			veh[i].arrive_flag = 1;
			res = true;
		}
		break;
	case 6:
	case 7:
	case 8:
		if(veh[i].x-VEH_LENGTH/2 <= RIGHT_STOP_LINE && veh[i].arrive_flag==0)
		{
			veh[i].arrive_flag = 1;
			res = true;
		}
		break;
	case 9:
	case 10:
	case 11:
		if(veh[i].y+VEH_LENGTH/2 >= TOP_STOP_LINE && veh[i].arrive_flag==0)
		{
			veh[i].arrive_flag = 1;
			res = true;
		}
		break;
	default:
		res = false;
	}
	return res;
}

/*���³����ٶ�*/
void update_v()
{
	int i;
	for(i=1;i<=num;i++)
	{
		if(veh[i].flag == 0) continue; //������Ч��
		//�����ٶ�
		veh[i].v += veh[i].a;

		if(veh[i].v > MAX_V)	//���ܳ�������ٶ�
			veh[i].v = MAX_V;

		if(veh[i].v < 0)	//�ٶȲ���Ϊ��
			veh[i].v = 0;

		if(arrive_intersection(i)&&!signal[veh[i].lane/3])
			veh[i].v = 0;
	}
}

/*���³���λ��*/
void update_pos()
{
	int i;
	for(i=1;i<=num;i++)
	{
		if(veh[i].flag == 0) continue;	//������Ч��
		
		if(veh[i].turn_time>0)	//����ת��
		{
			veh[i].ang += veh[i].omega;
			if(veh[i].lane%3==0)//��ת����
			{
				veh[i].x = turn_circle[veh[i].lane].x +	(int)(turn_rad[veh[i].lane]*sin(veh[i].ang));
				veh[i].y = turn_circle[veh[i].lane].y +	(int)(turn_rad[veh[i].lane]*cos(veh[i].ang));
			}

			if(veh[i].lane%3==2)//��ת����
			{
				veh[i].x = turn_circle[veh[i].lane].x -	(int)(turn_rad[veh[i].lane]*cos(PI/2-veh[i].ang));
				veh[i].y = turn_circle[veh[i].lane].y -	(int)(turn_rad[veh[i].lane]*sin(PI/2-veh[i].ang));
			}
			veh[i].turn_time--;
			if(veh[i].turn_time ==0)	//ת�����
			{
				veh[i].leave_flag = 1;
				if(veh[i].lane%3==0)	//��ת�任����
					veh[i].lane = (veh[i].lane + 3) % 12;//�任��������ת��0->3,3->6,6->9,9->0��
				if(veh[i].lane%3==2)	//��ת��ɱ任����
					veh[i].lane = (veh[i].lane + 12 - 3) % 12;//�任��������ת����2->11,5->2,8->5,11->8
				
				/*����ת��ƫ��*/
				int lane = veh[i].lane;
				if(lane==0||lane==1||lane==2||lane==6||lane==7||lane==8)
					veh[i].y = lane_center[lane];
				else
					veh[i].x = lane_center[lane];
			}
		}
		else	//����δת�䳵����λ��
		{
			if(veh[i].lane == 0||veh[i].lane==1||veh[i].lane==2)
				veh[i].x += (int)veh[i].v;
			else if(veh[i].lane==3||veh[i].lane == 4||veh[i].lane==5)
				veh[i].y -= (int)veh[i].v;
			else if(veh[i].lane == 6||veh[i].lane == 7||veh[i].lane == 8)
				veh[i].x -= (int)veh[i].v;
			else if(veh[i].lane == 9||veh[i].lane == 10||veh[i].lane == 11)
				veh[i].y += (int)veh[i].v;
//			veh[i].x += (int)(veh[i].v * cos(veh[i].ang) + 0.5);	//����x����
//			veh[i].y += -1 * (int)(veh[i].v * sin(veh[i].ang) + 0.5);	//����y���꣬ǰ�����-1��������Ļ����ϵ����Ϊ������
		}

		if(arrive_intersection(i))
		{
			if(veh[i].lane%3!=1)		//ת�䳵��
			{
				if(veh[i].lane%3==0)	//��ת����
				{
					veh[i].turn_time = (int)(LEFT_TURN_DIS / veh[i].v + 0.5);
					veh[i].omega = PI / 2 / veh[i].turn_time;
				}
				else					//��ת����
				{
					veh[i].turn_time = (int)(RIGHT_TURN_DIS / veh[i].v + 0.5);
					veh[i].omega = -1 * PI / 2 / veh[i].turn_time;
				}
//				veh[i].a = 0;	//��ת����ٶ�Ϊ0
				veh[i].arrive_flag = 1;	//���õ��ｻ��ڱ�־Ϊ1
			}
		}
		//����ʻ����Ļ�����Ϊ��Ч
		if(veh[i].x>SCREEN_LENGTH || veh[i].x<0 || veh[i].y<0 || veh[i].y>SCREEN_WIDTH)
			veh[i].flag = 0;
	}
}


int main()
{
	int time_green = TIME_EAST, time_red = 0, time_yellow = TIME_YELLOW;
	initgraph(SCREEN_LENGTH, SCREEN_WIDTH);
	init_intersection();
	set_light();
	time_total = 0;
	num = 1;
	while(time_total++<MAXTIME)
	{
		set_light();
		if(time_total%7==0)
			create_vehicle(++num);	//ÿ�����һ���³�
		update_pos();	//�������г�λ��
		refresh_vehicles();
		update_v();		//�������г��ٶ�
		update_a();
	}
	getch();
	closegraph();
	return 0;
}