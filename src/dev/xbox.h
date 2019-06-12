#ifndef XBOX_H
#define XBOX_H

#define XBOX_TYPE_BUTTON 0x01
#define XBOX_TYPE_AXIS 0x02

#define XBOX_BUTTON_A 0x00
#define XBOX_BUTTON_B 0x01
#define XBOX_BUTTON_X 0x02
#define XBOX_BUTTON_Y 0x03
#define XBOX_BUTTON_LB 0x04
#define XBOX_BUTTON_RB 0x05
#define XBOX_BUTTON_START 0x06
#define XBOX_BUTTON_BACK 0x07
#define XBOX_BUTTON_HOME 0x08
#define XBOX_BUTTON_LO 0x09
#define XBOX_BUTTON_RO 0x0a

#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00

#define XBOX_AXIS_LX 0x00
#define XBOX_AXIS_LY 0x01
#define XBOX_AXIS_RX 0x03
#define XBOX_AXIS_RY 0x04
#define XBOX_AXIS_LT 0x02
#define XBOX_AXIS_RT 0x05
#define XBOX_AXIS_XX 0x06
#define XBOX_AXIS_YY 0x07

#define XBOX_AXIS_VAL_UP -32767
#define XBOX_AXIS_VAL_DOWN 32767
#define XBOX_AXIS_VAL_LEFT -32767
#define XBOX_AXIS_VAL_RIGHT 32767

#define XBOX_AXIS_VAL_MIN -32767
#define XBOX_AXIS_VAL_MAX 32767
#define XBOX_AXIS_VAL_MID 0x00

typedef struct XBOX_t
{
	int fd;
	char filename[20];

	int time;

	unsigned char a;	 //Button -A
	unsigned char b;	 //Button -B
	unsigned char x;	 //Button -X
	unsigned char y;	 //Button -Y
	unsigned char start; //Button -Start
	unsigned char back;  //Button -Back
	unsigned char home;  //Button -Home
	unsigned char lo;	//Joystick Left Button
	unsigned char ro;	//Joystick Right Button
	unsigned char lb;	//Switch -LB
	unsigned char rb;	//Switch -RB

	int lx; //Joystick Left Axis-x
	int ly; //Joystick Left Axis-y
	int rx; //Joystick Right Axis-x
	int ry; //Joystick Right Axis-y
	int lt; //Switch -LT
	int rt; //Switch -RT
	int xx; //Crossing -X
	int yy; //Crossing -Y

	float lx_f; //Left Axis-x Normalized Factor
	float ly_f; //Left Axis-y Normalized Factor
	float rx_f; //Right Axis-x Normalized Factor
	float ry_f; //Right Axis-y Normalized Factor
	float lt_f; //Switch -LT Normalized Factor
	float rt_f; //Switch -RT Normalized Factor
	int xx_l;   //Crossing -X Logic Value
	int yy_l;   //Crossing -Y Logic Value

	signed char a_edge;
	signed char b_edge;
	signed char x_edge;
	signed char y_edge;
	signed char start_edge;
	signed char back_edge;
	signed char home_edge;
	signed char lo_edge;
	signed char ro_edge;
	signed char lb_edge;
	signed char rb_edge;
	signed char xx_edge;
	signed char yy_edge;

} XBOX_t;

int XBOX_Init(XBOX_t *xbox);
int XBOX_Read(XBOX_t *xbox);
void XBOX_Normal(XBOX_t *xbox);
void XBOX_Edge(XBOX_t *xbox);
void XBOX_Close(XBOX_t *xbox);
void XBOX_DispAll(XBOX_t *xbox);

#endif
