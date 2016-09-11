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

//默认设置
#define SERVO_NUM (10)
static int DEVICE = 0;
static int BAUDNUM  = 1;
static int PosRU[9][5], PosRD[9][5];
//static int PosLU[9][5], PosLD[9][5];

//子功能函数
int set_arm_byte(char dir, int address, int *value);
int set_arm_word(char dir, int address, int *value);
int wait_arm_stop_exten(char dir, int exten); //等待运动停止
int hit_and_hit(char dir);
int move_1_9();

//架构函数
int initial_sys(int device, int bandnum); //系统初始化
int default_pos();                        //初始化位置
int relax_arm(char dir);                  //放松手臂
void print_temprature(char dir);          //打印此时的温度

int main()
{
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
    
    //getchar();
    //hit_and_hit('L');
    move_1_9();
    
    print_temprature('L');
    printf("relax ...");
    getchar();
    //松掉刚度
    if(1 != relax_arm('L'))
        return 0;
    
    return 1;
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

//测试击打
int hit_and_hit(char dir)
{
    int value[5] = {120, 50 + 50, 50 + 50, 50 + 50, 100};
    int pos[5] = {770, 453, 518, 754, 210};
    int i, diffK[10] = {3, 8, 12, 16, 20, 24, 28, 31, 34, 39};
    int posK;
    
    //设置速度
    set_arm_word(dir, Moving_Speed, value);
    
    //运动到目标值
    set_arm_word(dir, Goal_Position, pos);
    wait_arm_stop_exten(dir, 8);
    
    printf("hit ...\n");
    getchar();
    set_one_servo_word(10, Moving_Speed, 359);
    for(i = 0; i < 10; i++) { 
        posK = pos[4] + diffK[i];
        set_one_servo_word(10, Goal_Position, posK);
        delay_us(5 * 1000); //50ms
        printf("posK:%d\n", posK);
    }
    set_one_servo_word(10, Moving_Speed, 559);
    for(i = 8; i >= 0; i--) {
        posK = pos[4] + diffK[i];
        set_one_servo_word(10, Goal_Position, posK);
        delay_us(5 * 1000);
        printf("posK:%d\n", posK);
    }
    //敲击
    
    return 1;
}


//测试左臂从琴键序1运动到琴键序9：看其能否平滑运动
int move_1_9()
{
    int i, j, posK;
    //int pos1[5] = {770, 453, 518, 754, 210};
    //int pos2[5] = {756, 353, 670, 716, 301};
    int pos1[5] = {799, 460, 560, 766, 217};
    /*int diff[7][5] = {{-2, -14, 22, -5, 13},
                      {-4, -28, 44, -10,26},
                      {-6, -42, 66, -15,39},
                      {-8, -56, 88, -20,52},
                      {-10,-70, 110,-25,65},
                      {-12,-84, 132,-30,78},
                      {-14,-100,152,-38,91}};
    */
    int diff[7][5] = {{0, -3, 1, 0, 0},
                      {0, -6, 2, 0, 0},
                      {0, -9, 3, 0, 0},
                      {0, -12,4, 0, 0},
                      {0, -15,5, 0, 0},
                      {0, -18,6, 0, 0},
                      {0, -23,10,0, 0}};
    //int value[5] = {14, 100, 152, 38, 91};
    int value[5] = {2, 23, 10, 2, 2};
    //运动到起始点
    set_arm_word('L', Goal_Position, pos1);
    wait_arm_stop_exten('L', 8);
    
    printf("move smooth ...\n");
    getchar();
    
    //设置速度
    set_arm_word('L', Moving_Speed, value);
    for(i = 0; i < 7; i++) {
        for(j = 0; j < 5; j++) {
            posK = diff[i][j] + pos1[j];
            set_one_servo_word(j + 6, Goal_Position, posK);
            delay_us(1 * 1000);
        }
        //delay_us(100);
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
        result = set_one_servo_word(i+1, Moving_Speed, 60);
        if(0 == result) {
            printf("err:default_pos set speed failed...\n");
            return 0;
        }
    }
    
    //运动到目标位置
    valueR[0] = 719; valueR[1] = 342; valueR[2] = 764; 
    valueR[3] = 711; valueR[4] = 214;
    set_one_servo_word(10, Goal_Position, valueR[4]);
    delay_us(500000);
    for(i = 1; i <= 4; i++) {
        //result = set_arm_word('R', Goal_Position, valueR);
        set_one_servo_word(i + 5, Goal_Position, valueR[i-1]);
    }
    
    return 1;
}

//放松手臂
int relax_arm(char dir)
{
    int i;
    
    //放松左手
    if('L' == dir) {
        for(i = 6; i <= 10; i++)
            set_one_servo_byte(i, Torque_Enable, 0);
    }
    else if('R' == dir) {
        //放松右手
        for(i = 1; i <= 5; i++)
            set_one_servo_byte(i, Torque_Enable, 0);
    }
    else{
        printf("relax_arm failed ...\n");
        return 0;
    }
    
    return 1;
}

//打印此时的温度
void print_temprature(char dir)
{
    int i, base = 1, temprature;
    
    if('L' == dir) 
        base = 1;
    else
        base = 6;
        
    for(i = 0; i < SERVO_NUM/2; i++) {
        temprature = get_one_servo_byte(i+base, 43);
        printf("the id(%d) temprature(%d)\n", i+base, temprature);
    }
}
