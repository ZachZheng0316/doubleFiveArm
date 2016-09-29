#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <math.h>
#include "AX12A.h"
#include "dmath.h"

#define SERVO_NUM (10)
static int DEVICE = 0;
static int BAUDNUM  = 1;

//子功能函数
int set_arm_byte(char dir, int address, int *value);
int set_arm_word(char dir, int address, int *value);
int wait_arm_stop_exten(char dir, int exten); //等待运动停止

int initial_sys(int device, int bandnum); //系统初始化
int default_pos();                        //初始化位置

int main()
{
    int value[5], id;
    char choice;
    
    if(1 != initial_sys(DEVICE, BAUDNUM)) {
        printf("err:initial failed ...\n");
        getchar();
        return 0;
    }
    
    //位置初始化
    if(1 != default_pos()) {
        printf("err:default_pos ...\n");
        getchar();
        return 0;
    }
    wait_arm_stop_exten('L', 8);

    printf("welcome to control arms with servos ...\n");
    while(1) {
        printf("please choose control mode:q: exit, s:sigle, m:arms\n"); 
        fflush(stdout);
        if(0 == scanf("%c", &choice)) return 0;
        if('q' == choice) {
            printf("being exiting ...\n");
            set_one_servo_word(1, Torque_Enable, 0);
            set_one_servo_word(2, Torque_Enable, 0);
            set_one_servo_word(3, Torque_Enable, 0);
            set_one_servo_word(4, Torque_Enable, 0);
            set_one_servo_word(5, Torque_Enable, 0);
            set_one_servo_word(6, Torque_Enable, 0);
            set_one_servo_word(7, Torque_Enable, 0);
            set_one_servo_word(8, Torque_Enable, 0);
            set_one_servo_word(9, Torque_Enable, 0);
            set_one_servo_word(10, Torque_Enable, 0);
            break;
        }
        else if('s' == choice) {
            printf("please intput servo id:"); fflush(stdout);
            if(0 == scanf("%d", &id)) return 0;
            value[0] = get_one_servo_word(id, Present_Position);
            value[1] = get_one_servo_word(id, 40);
            printf("%d servo pre-position %d Load %d\n", id, value[0], value[1]%1024);
            printf("please input servo goal-position:");
            if(0 == scanf("%d", &value[0])) return 0;
            set_one_servo_word(id, Goal_Position, value[0]);
            wait_for_one_servo(id);
        }
        else if('m' == choice) {
            //获取当前位置
            
            value[0] = get_one_servo_word(6, Present_Position);
            value[1] = get_one_servo_word(7, Present_Position);
            value[2] = get_one_servo_word(8, Present_Position);
            value[3] = get_one_servo_word(9, Present_Position);
            value[4] = get_one_servo_word(10, Present_Position);
            printf("servo 6 pre-position %d\n", value[0]);
            printf("servo 7 pre-position %d\n", value[1]);
            printf("servo 8 pre-position %d\n", value[2]);
            printf("servo 9 pre-position %d\n", value[3]);
            printf("servo 10 pre-position %d\n", value[4]);
             printf("please input servo goal-position:");
             if(0 == scanf("%d %d %d %d %d", &value[0], &value[1],
                &value[2], &value[3], &value[4])) return 0;
            set_one_servo_word(6, Goal_Position, value[0]);
            set_one_servo_word(7, Goal_Position, value[1]);
            set_one_servo_word(8, Goal_Position, value[2]);
            set_one_servo_word(9, Goal_Position, value[3]);
            set_one_servo_word(10, Goal_Position, value[4]);
            
            /*
            value[0] = get_one_servo_word(1, Present_Position);
            value[1] = get_one_servo_word(2, Present_Position);
            value[2] = get_one_servo_word(3, Present_Position);
            value[3] = get_one_servo_word(4, Present_Position);
            value[4] = get_one_servo_word(5, Present_Position);
            printf("servo 1 pre-position %d\n", value[0]);
            printf("servo 2 pre-position %d\n", value[1]);
            printf("servo 3 pre-position %d\n", value[2]);
            printf("servo 4 pre-position %d\n", value[3]);
            printf("servo 5 pre-position %d\n", value[4]);
             printf("please input servo goal-position:");
             if(0 == scanf("%d %d %d %d %d", &value[0], &value[1],
                &value[2], &value[3], &value[4])) return 0;
            set_one_servo_word(1, Goal_Position, value[0]);
            set_one_servo_word(2, Goal_Position, value[1]);
            set_one_servo_word(3, Goal_Position, value[2]);
            set_one_servo_word(4, Goal_Position, value[3]);
            set_one_servo_word(5, Goal_Position, value[4]);
            */
        }
        else{
        
        }
    }

    return 0;
}

int set_arm_byte(char dir, int address, int *value)
{
    int i, base = 1, result;
    
    //如果是左臂
    if('L' == dir)
        base = 6;
    else if('R' == dir) //如果是右臂
        base = 1;
    else
        return 0;
    
    for(i = 0; i < SERVO_NUM/2; i++) {
        result = set_one_servo_byte(i+base, address, value[i]);
        if(0 == result) {
            printf("err:set arm byte address:%d failed...\n", address);
            return 0;
        }
    }
    
    return 1;
}


int set_arm_word(char dir, int address, int *value)
{
    int i, base = 1, result;
    
    //如果是左臂
    if('L' == dir)
        base = 6;
    else if('R' == dir) //如果是右臂
        base = 1;
    else
        return 0;
    
    for(i = 0; i < SERVO_NUM/2; i++) {
        result = set_one_servo_word(i+base, address, value[i]);
        if(0 == result) {
            printf("err:set arm word address:%d failed...\n", address);
            return 0;
        }
    }
    
    return 1;
}

//等待运动停止
int wait_arm_stop_exten(char dir, int exten)
{
    int i, base = 1, diff;
    int goal[5], pre[5], flag = 0;
    
    if('L' == dir)
        base = 6;
    else if('R' == dir)
        base = 1;
    else
        return 0;
    
    //获得目标刻度
    for(i = 0; i < SERVO_NUM/2; i++) 
        goal[i] = get_one_servo_word(i+base, Goal_Position);
    
    while(flag < 5) {
        flag = 0;
    
        //获得当前目标刻度
        for(i = 0; i < SERVO_NUM/2; i++)
            pre[i] = get_one_servo_word(i+base, Present_Position);
            
        for(i = 0; i < SERVO_NUM/2; i++) {
            diff = goal[i] - pre[i];
            if(abs(diff) <= exten)
                flag += 1;
        }
    }
    
    return 1;
}

//系统初始化
int initial_sys(int device, int bandnum)
{
    int i;
    
    if(dxl_initialize(device, bandnum) == 0) {
        printf("Failed to open USB2Dynamixel!\n");
        printf("Press Enter key to terminate ...\n");
        getchar();
        
        return 0;
    }
    else {
        printf("Succeed to open USN2Dynamixel!\n");
    }
    
    //参数初始化
    //1.slope
    for(i = 0; i < SERVO_NUM; i++) {
        set_one_servo_byte(i+1, CW_Slope, 64);
        set_one_servo_byte(i+1, CCW_Slope, 64);
    }
    set_one_servo_byte(2, CW_Slope, 32);
    set_one_servo_byte(2, CCW_Slope, 32);
    set_one_servo_byte(7, CW_Slope, 32);
    set_one_servo_byte(7, CCW_Slope, 32);
        
        
    //激活
    for(i = 0; i < SERVO_NUM; i++) {
        set_one_servo_byte(i +1, Torque_Enable, 1);
    }
        
    
    return 1;
}

//初始化位置
int default_pos()
{
    int valueR[5];
    int i, result;
    
    //设置速度
    for(i = 0; i < SERVO_NUM; i++) {
        result = set_one_servo_word(i+1, Moving_Speed, 50);
        if(0 == result) {
            printf("err:default_pos set speed failed...\n");
            return 0;
        }
    }
    
    //运动到目标位置
    
    valueR[0] = 796; valueR[1] = 422; valueR[2] = 599; 
    valueR[3] = 695; valueR[4] = 209;
    set_one_servo_word(10, Goal_Position, valueR[4]);
    delay_us(500000);
    for(i = 1; i <= 4; i++) {
        //result = set_arm_word('R', Goal_Position, valueR);
        set_one_servo_word(i+5, Goal_Position, valueR[i-1]);
    }
    /*
    valueR[0] = 334; valueR[1] = 616; valueR[2] = 614; 
    valueR[3] = 304; valueR[4] = 763;
    set_one_servo_word(5, Goal_Position, valueR[4]);
    delay_us(500000);
    for(i = 1; i <= 4; i++) {
        //result = set_arm_word('R', Goal_Position, valueR);
        set_one_servo_word(i, Goal_Position, valueR[i-1]);
    }*/
    
    return 1;
}

