#include <stdio.h>
#include <graphics.h>
#include <stdlib.h>
#include <math.h>
#include <conio.h>
#include <time.h>

#define PI 3.1415926
#define SCREEN_LENGTH	1920
#define SCREEN_WIDTH	1080

//以下涉及到长度的单位均为像素，每像素代表实际长度0.45米
#define MAXNUM	100000	//最大车辆数目
#define MAXTIME	100001
#define MAX_A	0.7	//最大加速度，相当于4.4m/s2.
#define MIN_A	-1	//最小加速度，即刹车加速度，相当于-4.4m/s2.
#define MAX_V	5	//车速1-5档，分别相当于(16.2km/h, 32.4km/s, 48.6km/s, 64.8km/h, 81km/h)
#define MIN_V	0	//不能倒车
#define VEH_LENGTH	10	//车长，约10*0.45=4.5米
#define VEH_WIDTH	4	//车宽，约4*0.45=1.8米
#define MIN_DIS 3	//最小车间距
#define MAX_DIS 20	//最大车距（当车距大于此值时，车辆加速，直至达到最大速度或达到最小车间距
#define FREQU	100	//刷新频率为100ms
#define LANE_WIDTH	20	//车道宽度，约16*0.45=7.2米，比实际车道宽，是为了放大交叉口以便更清晰
#define ARC_RAD		16	//交叉口圆角弧度半径
#define CROSS_X	(SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD)
#define CROSS_Y	(SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD)
#define CROSS_WIDTH	(3*LANE_WIDTH)
#define CROSS_LENGTH	(3*LANE_WIDTH)

#define LEFT_TURN_DIS	(PI/2*(ARC_RAD + 7*LANE_WIDTH/2))	//左转车辆转弯距离
#define RIGHT_TURN_DIS	(PI/2*(ARC_RAD + LANE_WIDTH/2))		//右转车辆转弯距离
#define HAVE_SIGNAL	true
#define RAD_LIGHT	5	//信号灯半径

//四条停车线的X(Y)坐标
#define LEFT_STOP_LINE		SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD
#define RIGHT_STOP_LINE		SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD
#define TOP_STOP_LINE		SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD
#define BOTTOM_STOP_LINE	SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD

//红绿灯时间
#define TIME_EAST	200	//设置东西方向绿灯时间为20秒
#define TIME_SOUTH	200	//设置南北方向绿灯时间为20秒
#define TIME_YELLOW	50	//清空时间，即黄灯时间

typedef struct
{
	int x;
	int y;
} point;	//定义点

int time_total = 0;	//当前时间步
int num = 0;	//车辆数目
int cur_temp[12];	//临时存放每车道的当前车序号
COLORREF color[6] = {GREEN, LIGHTRED, YELLOW, LIGHTBLUE, CYAN, RED};

bool signal[4] = {true, false, true, false};	//四个红绿灯，true表示绿灯，false表示红灯
point red_light[4] = {{LEFT_STOP_LINE+RAD_LIGHT, SCREEN_WIDTH/2+RAD_LIGHT},
{SCREEN_LENGTH/2+RAD_LIGHT, BOTTOM_STOP_LINE-RAD_LIGHT},
{RIGHT_STOP_LINE-RAD_LIGHT, SCREEN_WIDTH/2-RAD_LIGHT},
{SCREEN_LENGTH/2-RAD_LIGHT,TOP_STOP_LINE+RAD_LIGHT}};
point green_light[4] = {{LEFT_STOP_LINE+RAD_LIGHT, SCREEN_WIDTH/2+3*RAD_LIGHT},
{SCREEN_LENGTH/2+3*RAD_LIGHT, BOTTOM_STOP_LINE-RAD_LIGHT},
{RIGHT_STOP_LINE-RAD_LIGHT, SCREEN_WIDTH/2-3*RAD_LIGHT},
{SCREEN_LENGTH/2-3*RAD_LIGHT,TOP_STOP_LINE+RAD_LIGHT}};


//bool cross_occ[CROSS_LENGTH][CROSS_WIDTH];	//标志交叉口内该像素位置是否被占用

typedef struct
{
	int x;		//x坐标，小车中心点的X坐标
	int y;		//y坐标，小车中心点的Y坐标
	double v;	//速度，为标量
	double a;	//加速度，为标量。使用需配合行驶角
	COLORREF c;	//颜色
	int lane;	//车道编号
	int pre_veh;//当前车道的上一辆车的索引
	int flag;	//标记车辆状态，0为无效，1为有效。无效车辆是已经驶出该区域的车辆
	double ang;	//前轮行驶角度，向右为0度，顺时针增加至2PI.
	int turn_time;
	bool arrive_flag;	//进入交叉口标志，进入后为1
	bool leave_flag;	//出交叉口标志，出去后为1
	double omega;		//转弯角速度
} vehicle;

vehicle veh[MAXNUM];	//保存车辆信息

//各车道中心线
int lane_center[12] = {SCREEN_WIDTH/2+LANE_WIDTH/2, SCREEN_WIDTH/2+3*LANE_WIDTH/2, SCREEN_WIDTH/2+5*LANE_WIDTH/2,
					SCREEN_LENGTH/2+LANE_WIDTH/2,SCREEN_LENGTH/2+3*LANE_WIDTH/2,SCREEN_LENGTH/2+5*LANE_WIDTH/2, 
					SCREEN_WIDTH/2-LANE_WIDTH/2,SCREEN_WIDTH/2-3*LANE_WIDTH/2,SCREEN_WIDTH/2-5*LANE_WIDTH/2,
					SCREEN_LENGTH/2-LANE_WIDTH/2,SCREEN_LENGTH/2-3*LANE_WIDTH/2,SCREEN_LENGTH/2-5*LANE_WIDTH/2,};

//各车道转弯圆心坐标,坐标为0,0表示直行车道
point turn_circle[12] = \
{
	{LEFT_STOP_LINE,TOP_STOP_LINE},{0,0},{LEFT_STOP_LINE,BOTTOM_STOP_LINE},
	{LEFT_STOP_LINE,BOTTOM_STOP_LINE},{0,0},{RIGHT_STOP_LINE, BOTTOM_STOP_LINE},
	{RIGHT_STOP_LINE, BOTTOM_STOP_LINE},{0,0},{RIGHT_STOP_LINE, TOP_STOP_LINE},
	{RIGHT_STOP_LINE, TOP_STOP_LINE},{0,0},{LEFT_STOP_LINE,TOP_STOP_LINE},
};

//各车道转弯半径，0表示直行车道
int turn_rad[12] = \
{ARC_RAD+7*LANE_WIDTH/2,0,ARC_RAD+LANE_WIDTH/2,ARC_RAD+7*LANE_WIDTH/2,0,ARC_RAD+LANE_WIDTH/2,ARC_RAD+7*LANE_WIDTH/2,0,ARC_RAD+LANE_WIDTH/2,ARC_RAD+7*LANE_WIDTH/2,0,ARC_RAD+LANE_WIDTH/2};
/*12个车道的车辆初始位置*/
point init_pos[12] = \
{
	{0, lane_center[0]},	//车道0
	{0, lane_center[1]}, //车道1
	{0, SCREEN_WIDTH/2+5*LANE_WIDTH/2},//车道2

	{SCREEN_LENGTH/2+LANE_WIDTH/2, SCREEN_WIDTH}, //车道3
	{SCREEN_LENGTH/2+3*LANE_WIDTH/2, SCREEN_WIDTH}, //车道4
	{SCREEN_LENGTH/2+5*LANE_WIDTH/2, SCREEN_WIDTH},//车道5

	{SCREEN_LENGTH, SCREEN_WIDTH/2-LANE_WIDTH/2}, //车道6
	{SCREEN_LENGTH, SCREEN_WIDTH/2-3*LANE_WIDTH/2},
	{SCREEN_LENGTH, SCREEN_WIDTH/2-5*LANE_WIDTH/2},//车道8

	{SCREEN_LENGTH/2-LANE_WIDTH/2, 0}, //车道9
	{SCREEN_LENGTH/2-3*LANE_WIDTH/2, 0}, //车道10
	{SCREEN_LENGTH/2-5*LANE_WIDTH/2, 0},//车道11
};

//根据signal数组的值来设置红绿灯。signal[0],[1],[2],[3]分别表示左下右上四个方向的灯。
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
			//设置灯灭
			clearcircle(red_light[i].x, red_light[i].y, RAD_LIGHT);
			clearcircle(green_light[i].x, green_light[i].y, RAD_LIGHT);

			setfillcolor(GREEN);	//设置绿灯亮
			solidcircle(green_light[i].x, green_light[i].y, RAD_LIGHT);
		}
		else
		{
			//设置灯灭
			clearcircle(green_light[i].x, green_light[i].y, RAD_LIGHT);
			clearcircle(red_light[i].x, red_light[i].y, RAD_LIGHT);

			setfillcolor(RED);	//设置红灯亮
			solidcircle(red_light[i].x, red_light[i].y, RAD_LIGHT);
		}
	}
}
void init_intersection()
{
	setbkcolor(DARKGRAY);	//设置背景颜色
	cleardevice();
	setlinestyle(PS_SOLID);
	//画中线
	setlinecolor(YELLOW);
	line(0, SCREEN_WIDTH/2, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2);	//横1
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2, SCREEN_LENGTH, SCREEN_WIDTH/2);//横2
	line(SCREEN_LENGTH/2, 0, SCREEN_LENGTH/2, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	//纵1
	line(SCREEN_LENGTH/2, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2, SCREEN_WIDTH);//纵2
	setlinecolor(WHITE);

	//画横线
	line(0, SCREEN_WIDTH/2-3*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH);
	line(0, SCREEN_WIDTH/2+3*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2-3*LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2+3*LANE_WIDTH);

	//画纵线
	line(SCREEN_LENGTH/2-3*LANE_WIDTH, 0, SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH, 0, SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	

	line(SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH);	//纵1
	line(SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH);	//纵1

	//画交叉口相交弧，四条弧为顺时针1,2,3,4
	arc(SCREEN_LENGTH/2-3*LANE_WIDTH-2*ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH-2*ARC_RAD, SCREEN_LENGTH/2-3*LANE_WIDTH,SCREEN_WIDTH/2-3*LANE_WIDTH, 1.5*PI, 2*PI);
	arc(SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-2*ARC_RAD, SCREEN_LENGTH/2+3*LANE_WIDTH+2*ARC_RAD,SCREEN_WIDTH/2-3*LANE_WIDTH, PI, 1.5*PI);
	arc(SCREEN_LENGTH/2-3*LANE_WIDTH-2*ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH,SCREEN_WIDTH/2+3*LANE_WIDTH+2*ARC_RAD, 0, 0.5*PI);
	arc(SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH, SCREEN_LENGTH/2+3*LANE_WIDTH+2*ARC_RAD,SCREEN_WIDTH/2+3*LANE_WIDTH+2*ARC_RAD, 0.5*PI, PI);

	//画停车线 分别为 左，右，上，下
	line(SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2-3*LANE_WIDTH, SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2+3*LANE_WIDTH);

	line(SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);
	line(SCREEN_LENGTH/2-3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2+3*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD);

	//画车道
	setlinestyle(PS_DASH);
	//左四条
	line(0, SCREEN_WIDTH/2-2*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2-2*LANE_WIDTH);
	line(0, SCREEN_WIDTH/2-LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2-LANE_WIDTH);
	line(0, SCREEN_WIDTH/2+LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2+LANE_WIDTH);
	line(0, SCREEN_WIDTH/2+2*LANE_WIDTH, SCREEN_LENGTH/2-3*LANE_WIDTH-ARC_RAD, SCREEN_WIDTH/2+2*LANE_WIDTH);
	//右四条
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2-2*LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2-2*LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2-LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2-LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2+LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2+LANE_WIDTH);
	line(SCREEN_LENGTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_WIDTH/2+2*LANE_WIDTH, SCREEN_LENGTH, SCREEN_WIDTH/2+2*LANE_WIDTH);
	//上四条
	line(SCREEN_LENGTH/2-LANE_WIDTH, 0, SCREEN_LENGTH/2-LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	
	line(SCREEN_LENGTH/2-2*LANE_WIDTH, 0, SCREEN_LENGTH/2-2*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	
	line(SCREEN_LENGTH/2+LANE_WIDTH, 0, SCREEN_LENGTH/2+LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	
	line(SCREEN_LENGTH/2+2*LANE_WIDTH, 0, SCREEN_LENGTH/2+2*LANE_WIDTH, SCREEN_WIDTH/2-3*LANE_WIDTH-ARC_RAD);	
	//下四条	
	line(SCREEN_LENGTH/2-LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2-LANE_WIDTH, SCREEN_WIDTH);//纵2
	line(SCREEN_LENGTH/2-2*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2-2*LANE_WIDTH, SCREEN_WIDTH);//纵2
	line(SCREEN_LENGTH/2+LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2+LANE_WIDTH, SCREEN_WIDTH);//纵2
	line(SCREEN_LENGTH/2+2*LANE_WIDTH, SCREEN_WIDTH/2+3*LANE_WIDTH+ARC_RAD, SCREEN_LENGTH/2+2*LANE_WIDTH, SCREEN_WIDTH);//纵2

	//画信号灯外框
	rectangle(LEFT_STOP_LINE,SCREEN_WIDTH/2,LEFT_STOP_LINE+2*RAD_LIGHT,SCREEN_WIDTH/2+4*RAD_LIGHT);
	rectangle(SCREEN_LENGTH/2, BOTTOM_STOP_LINE-2*RAD_LIGHT,SCREEN_LENGTH/2+4*RAD_LIGHT,BOTTOM_STOP_LINE);
	rectangle(RIGHT_STOP_LINE-2*RAD_LIGHT,SCREEN_WIDTH/2-4*RAD_LIGHT,RIGHT_STOP_LINE,SCREEN_WIDTH/2);
	rectangle(SCREEN_LENGTH/2-4*RAD_LIGHT,TOP_STOP_LINE,SCREEN_LENGTH/2,TOP_STOP_LINE+2*RAD_LIGHT);

	setlinestyle(PS_SOLID);
}

/*画小车，小车必须已经被初始化*/
void vehicle_draw(int i)
{
	if(!veh[i].flag)
		return;
	//设置绘制坐标
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
	if(veh[i].turn_time==0)	//未转弯车辆
	{
		solidrectangle(left, top, right, bottom);
//		RECT r = {10, 10, 50, 50};
//		char s[5];
//		settextcolor(WHITE);
//		outtextxy(veh[i].x, veh[i].y, itoa(i,s,10));
	}
	else	//正在转弯的车辆
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

/*产生汽车*/
void create_vehicle(int index)
{
	int lane;
	srand((unsigned)time(NULL));
	if(index >= MAXNUM)
	{
		printf("车辆数目超标!\n");
		exit(1);
	}
	//初始化小车
	lane = rand() % 12;	//随机车道
	veh[index].c = color[rand()%6];
	veh[index].lane = lane;
	veh[index].x = init_pos[lane].x;
	veh[index].y = init_pos[lane].y;

	veh[index].v = (3+rand()%3);	//设置初始速度大小
	veh[index].a = 0;				//设置初始加速度大小

	veh[index].pre_veh = cur_temp[lane];	//记录前车索引
	if(veh[index].pre_veh==0)
		veh[index].v = MAX_V;
	veh[index].flag = 1;
	veh[index].ang = ((int)(lane/3)) * (PI / 2);
	cur_temp[lane] = index;		//设置当前车道当前车的索引
	veh[index].turn_time = 0;
	//绘制小车
	vehicle_draw(index);
}

void refresh_vehicles()
{
	int i;
	setwritemode(R2_XORPEN);	//异或模式，第二次绘制是用于擦除
	for(i=1;i<=num;i++)		//绘制
	{
		if(veh[i].flag == 0) continue;	//忽略无效车
		vehicle_draw(i);
	}
	Sleep(FREQU);
	for(i=1;i<=num;i++)		//擦除
	{
		if(veh[i].flag == 0) continue;	//忽略无效车
		vehicle_draw(i);
	}
}

/*更新加速度*/
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


		if(veh[i].flag == 0) continue;	//跳过无效车
		if(veh[i].v==MAX_V||veh[i].v==0)
			veh[i].a = 0;
		else
		{	
			double dif_v = veh[veh[i].pre_veh].v - veh[i].v;
			veh[i].a = dif_v < MAX_A ? dif_v : MAX_A;
		}
	}
}

/*判断到达交叉口的一瞬间*/
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

/*更新车辆速度*/
void update_v()
{
	int i;
	for(i=1;i<=num;i++)
	{
		if(veh[i].flag == 0) continue; //跳过无效车
		//更新速度
		veh[i].v += veh[i].a;

		if(veh[i].v > MAX_V)	//不能超过最大速度
			veh[i].v = MAX_V;

		if(veh[i].v < 0)	//速度不能为负
			veh[i].v = 0;

		if(arrive_intersection(i)&&!signal[veh[i].lane/3])
			veh[i].v = 0;
	}
}

/*更新车辆位置*/
void update_pos()
{
	int i;
	for(i=1;i<=num;i++)
	{
		if(veh[i].flag == 0) continue;	//跳过无效车
		
		if(veh[i].turn_time>0)	//正在转弯
		{
			veh[i].ang += veh[i].omega;
			if(veh[i].lane%3==0)//左转车辆
			{
				veh[i].x = turn_circle[veh[i].lane].x +	(int)(turn_rad[veh[i].lane]*sin(veh[i].ang));
				veh[i].y = turn_circle[veh[i].lane].y +	(int)(turn_rad[veh[i].lane]*cos(veh[i].ang));
			}

			if(veh[i].lane%3==2)//右转车辆
			{
				veh[i].x = turn_circle[veh[i].lane].x -	(int)(turn_rad[veh[i].lane]*cos(PI/2-veh[i].ang));
				veh[i].y = turn_circle[veh[i].lane].y -	(int)(turn_rad[veh[i].lane]*sin(PI/2-veh[i].ang));
			}
			veh[i].turn_time--;
			if(veh[i].turn_time ==0)	//转弯完成
			{
				veh[i].leave_flag = 1;
				if(veh[i].lane%3==0)	//左转变换车道
					veh[i].lane = (veh[i].lane + 3) % 12;//变换车道（左转车0->3,3->6,6->9,9->0）
				if(veh[i].lane%3==2)	//右转完成变换车道
					veh[i].lane = (veh[i].lane + 12 - 3) % 12;//变换车道（右转车：2->11,5->2,8->5,11->8
				
				/*修正转弯偏移*/
				int lane = veh[i].lane;
				if(lane==0||lane==1||lane==2||lane==6||lane==7||lane==8)
					veh[i].y = lane_center[lane];
				else
					veh[i].x = lane_center[lane];
			}
		}
		else	//更新未转弯车辆的位置
		{
			if(veh[i].lane == 0||veh[i].lane==1||veh[i].lane==2)
				veh[i].x += (int)veh[i].v;
			else if(veh[i].lane==3||veh[i].lane == 4||veh[i].lane==5)
				veh[i].y -= (int)veh[i].v;
			else if(veh[i].lane == 6||veh[i].lane == 7||veh[i].lane == 8)
				veh[i].x -= (int)veh[i].v;
			else if(veh[i].lane == 9||veh[i].lane == 10||veh[i].lane == 11)
				veh[i].y += (int)veh[i].v;
//			veh[i].x += (int)(veh[i].v * cos(veh[i].ang) + 0.5);	//更新x坐标
//			veh[i].y += -1 * (int)(veh[i].v * sin(veh[i].ang) + 0.5);	//更新y坐标，前面乘以-1是由于屏幕坐标系向下为正方向
		}

		if(arrive_intersection(i))
		{
			if(veh[i].lane%3!=1)		//转弯车辆
			{
				if(veh[i].lane%3==0)	//左转车辆
				{
					veh[i].turn_time = (int)(LEFT_TURN_DIS / veh[i].v + 0.5);
					veh[i].omega = PI / 2 / veh[i].turn_time;
				}
				else					//右转车辆
				{
					veh[i].turn_time = (int)(RIGHT_TURN_DIS / veh[i].v + 0.5);
					veh[i].omega = -1 * PI / 2 / veh[i].turn_time;
				}
//				veh[i].a = 0;	//设转弯加速度为0
				veh[i].arrive_flag = 1;	//设置到达交叉口标志为1
			}
		}
		//车辆驶出屏幕，标记为无效
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
			create_vehicle(++num);	//每秒产生一辆新车
		update_pos();	//更新所有车位置
		refresh_vehicles();
		update_v();		//更新所有车速度
		update_a();
	}
	getch();
	closegraph();
	return 0;
}