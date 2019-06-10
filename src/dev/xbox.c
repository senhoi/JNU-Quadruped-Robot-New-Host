#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include "xbox.h"

int XBOX_Init(XBOX_t *xbox)
{
    int file_index;
    int counter = 0;

    for (file_index = 0; file_index < 4; file_index++)
    {
        sprintf(xbox->filename, "/dev/input/js%d", file_index);
        if (access(xbox->filename, F_OK) != -1)
            xbox->fd = open(xbox->filename, O_RDONLY | O_NONBLOCK);
        else
            continue;

        if (xbox->fd == -1)
        {
            printf("Dev File: %s\n", xbox->filename);
            perror("Failed to open.\n");
        }
        else
        {
            printf("Success to open %s.\n", xbox->filename);
            printf("Press START to check the connection.\n");

            while (1)
            {
                XBOX_Read(xbox);
                if (xbox->start == 1)
                {
                    printf("Xbox connected!\n");
                    return 0;
                }
                else
                {
                    usleep(1000);
                    counter++;
                    if (counter > 10000)
                    {
                        counter = 0;
                        printf("Xbox response overtime.\n");
                        break;
                    }
                }
            }
        }
    }

    printf("Failed to connect with xbox.");
    return -1;
}

static float deadzone_func(float val, float deadzone)
{
    if (val < fabs(deadzone) && val > -fabs(deadzone))
        return 0;
    else if (val < -fabs(deadzone))
        return val + fabs(deadzone);
    else if (val > fabs(deadzone))
        return val - fabs(deadzone);
}

void XBOX_Normal(XBOX_t *xbox)
{
    if (xbox->xx > 0)
        xbox->xx_l = 1;
    else if (xbox->xx < 0)
        xbox->xx_l = -1;
    else
        xbox->xx_l = 0;

    if (xbox->yy > 0)
        xbox->yy_l = 1;
    else if (xbox->yy < 0)
        xbox->yy_l = -1;
    else
        xbox->yy_l = 0;

    xbox->lx_f = deadzone_func(xbox->lx, 3000) / XBOX_AXIS_VAL_MAX;
    xbox->ly_f = deadzone_func(xbox->ly, 3000) / XBOX_AXIS_VAL_MAX;
    xbox->lt_f = ((float)xbox->lt / XBOX_AXIS_VAL_MAX + 1.0f) / 2.0f;
    xbox->rx_f = deadzone_func(xbox->rx, 3000) / XBOX_AXIS_VAL_MAX;
    xbox->ry_f = deadzone_func(xbox->ry, 3000) / XBOX_AXIS_VAL_MAX;
    xbox->rt_f = ((float)xbox->rt / XBOX_AXIS_VAL_MAX + 1.0f) / 2.0f;
}

static int edge_func(int prev_val, int new_val)
{
    if (prev_val != new_val && prev_val == 0)
    {
        if (new_val > prev_val)
            return 1;
        else
            return -1;
    }
    else
        return 0;
}

static XBOX_t prev_xbox;

void XBOX_Edge(XBOX_t *xbox)
{

    xbox->a_edge = edge_func(prev_xbox.a, xbox->a);
    xbox->b_edge = edge_func(prev_xbox.b, xbox->b);
    xbox->x_edge = edge_func(prev_xbox.x, xbox->x);
    xbox->y_edge = edge_func(prev_xbox.y, xbox->y);
    xbox->lo_edge = edge_func(prev_xbox.lo, xbox->lo);
    xbox->ro_edge = edge_func(prev_xbox.ro, xbox->ro);
    xbox->lb_edge = edge_func(prev_xbox.lb, xbox->lb);
    xbox->rb_edge = edge_func(prev_xbox.rb, xbox->rb);
    xbox->start_edge = edge_func(prev_xbox.start, xbox->start);
    xbox->back_edge = edge_func(prev_xbox.back, xbox->back);
    xbox->home_edge = edge_func(prev_xbox.home, xbox->home);
    xbox->xx_edge = edge_func(prev_xbox.xx_l, xbox->xx_l);
    xbox->yy_edge = edge_func(prev_xbox.yy_l, xbox->yy_l);

    prev_xbox.a = xbox->a;
    prev_xbox.b = xbox->b;
    prev_xbox.x = xbox->x;
    prev_xbox.y = xbox->y;
    prev_xbox.lo = xbox->lo;
    prev_xbox.ro = xbox->ro;
    prev_xbox.lb = xbox->lb;
    prev_xbox.rb = xbox->rb;
    prev_xbox.start = xbox->start;
    prev_xbox.back = xbox->back;
    prev_xbox.home = xbox->home;
    prev_xbox.xx_l = xbox->xx_l;
    prev_xbox.yy_l = xbox->yy_l;
}

int XBOX_Read(XBOX_t *xbox)
{
    int len, type, number, value;
    struct js_event js;

    len = read(xbox->fd, &js, sizeof(struct js_event));
    if (len < 0)
        return -1;

    type = js.type;
    number = js.number;
    value = js.value;

    xbox->time = js.time;

    switch (type)
    {
    case JS_EVENT_BUTTON:
        switch (number)
        {
        case XBOX_BUTTON_A:
            xbox->a = value;
            break;

        case XBOX_BUTTON_B:
            xbox->b = value;
            break;

        case XBOX_BUTTON_X:
            xbox->x = value;
            break;

        case XBOX_BUTTON_Y:
            xbox->y = value;
            break;

        case XBOX_BUTTON_LB:
            xbox->lb = value;
            break;

        case XBOX_BUTTON_RB:
            xbox->rb = value;
            break;

        case XBOX_BUTTON_START:
            xbox->start = value;
            break;

        case XBOX_BUTTON_BACK:
            xbox->back = value;
            break;

        case XBOX_BUTTON_HOME:
            xbox->home = value;
            break;

        case XBOX_BUTTON_LO:
            xbox->lo = value;
            break;

        case XBOX_BUTTON_RO:
            xbox->ro = value;
            break;

        default:
            break;
        }
        break;

    case JS_EVENT_AXIS:
        switch (number)
        {
        case XBOX_AXIS_LX:
            xbox->lx = value;
            break;

        case XBOX_AXIS_LY:
            xbox->ly = value;
            break;

        case XBOX_AXIS_RX:
            xbox->rx = value;
            break;

        case XBOX_AXIS_RY:
            xbox->ry = value;
            break;

        case XBOX_AXIS_LT:
            xbox->lt = value;
            break;

        case XBOX_AXIS_RT:
            xbox->rt = value;
            break;

        case XBOX_AXIS_XX:
            xbox->xx = value;
            break;

        case XBOX_AXIS_YY:
            xbox->yy = value;
            break;

        default:
            break;
        }
        break;
    }

    return len;
}

void XBOX_DispAll(XBOX_t *xbox)
{
    //printf("XBOX File descriptor:%d\n", xbox->fd);
    //printf("\tA:%d\tB:%d\tX:%d\tY:%d\n", xbox->a, xbox->b, xbox->x, xbox->y);
    //printf("\tpA:%d\tpB:%d\tpX:%d\tpY:%d\n", prev_xbox.a, prev_xbox.b, prev_xbox.x, prev_xbox.y);
    //printf("\tLB:%d\tRB:%d\tLO:%d\tRO:%d\n", xbox->lb, xbox->rb, xbox->lo, xbox->ro);
    //printf("\tXX:%d\tYY:%d\tSTA:%d\tBCK:%d\tHOM:%d\n", xbox->xx, xbox->yy, xbox->start, xbox->back, xbox->home);
    //printf("\tLX:%d\tLY:%d\tLT:%d\tRX:%d\tRY:%d\tRT:%d\n", xbox->lx, xbox->ly, xbox->lt, xbox->rx, xbox->ry, xbox->rt);

    //printf("\tLX_F:%f\tLY_F:%f\tLT_F:%f\tRX_F:%f\tRY_F:%f\tRT_F:%f\n", xbox->lx_f, xbox->ly_f, xbox->lt_f, xbox->rx_f, xbox->ry_f, xbox->rt_f);

    if (xbox->a_edge == 1)
        printf("A Pressed!\n");
    if (xbox->b_edge == 1)
        printf("B Pressed!\n");
    if (xbox->x_edge == 1)
        printf("X Pressed!\n");
    if (xbox->y_edge == 1)
        printf("Y Pressed!\n");
    if (xbox->lo_edge == 1)
        printf("LO Pressed!\n");
    if (xbox->ro_edge == 1)
        printf("RO Pressed!\n");
    if (xbox->lb_edge == 1)
        printf("LB Pressed!\n");
    if (xbox->rb_edge == 1)
        printf("RB Pressed!\n");
    if (xbox->home_edge == 1)
        printf("HOME Pressed!\n");
    if (xbox->start_edge == 1)
        printf("START Pressed!\n");
    if (xbox->back_edge == 1)
        printf("BACK Pressed!\n");
    if (xbox->xx_edge == 1)
        printf("XX- Pressed!\n");
    if (xbox->xx_edge == -1)
        printf("XX+ Pressed!\n");
    if (xbox->yy_edge == 1)
        printf("YY- Pressed!\n");
    if (xbox->yy_edge == -1)
        printf("YY+ Pressed!\n");
}

int XBOX_Check(XBOX_t *xbox)
{
    if (access(xbox->filename, F_OK) == -1)
        return -1;
}

void XBOX_Close(XBOX_t *xbox)
{
    close(xbox->fd);
    return;
}
