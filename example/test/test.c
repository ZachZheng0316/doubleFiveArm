#include <stdio.h>
#include <stdlib.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <math.h>
#include "AX12A.h"
#include "dmath.h"

#define NOARMS (2) //表示机械臂的编号

//设置默认值
#define SERVO_NUM (10)
static int DEVICE = 0;
static int BAUDNUM = 1;
static int CurLArm[5];
static int CurRArm[5];
//static int TM1 = 290;
//static int TM2 = 2250;
static int LKeyPos[10][5];
static int RKeyPos[10][5];
static char LKeyPosPath[] = "LKeyPos.txt";
static char RKeyPosPath[] = "RKeyPos.txt";

//子功能函数
int set_arm_byte(char dir, int address, int *value);
int set_arm_word(char dir, int address, int *value);
int wait_arm_stop_exten(char dir, int exten);//等待运动停止
void raed_key_pos();                         //读取琴键的位置
void from_A_to_B(int num1, int num2);        //从A段时平滑的运动到B
void hit_and_hit(int num);                   //敲打音乐
void re_enable();                            //重新上刚度
void relax();                                //释放刚度

//框架函数
void initial_sys(int device, int bandnum);  //系统初始化
void set_default_pos();                     //设置初始化位置
void auto_test_key();                       //自动测试每个按键位置
void chose_key_test();                      //手动选择琴键
void set_posture();                         //设置姿态
int control_servo();                       //控制机械臂姿态
void relax_arm();                           //放松手臂
void print();                               //打印此时温度


int main()
{
    char choice;

    //系统初始化
    initial_sys(DEVICE, BAUDNUM);
    
    //设置到初始位置
    set_default_pos();
    
    printf("start ...\n");
    getchar();
    
    while(1) {
        printf("please choose the control mode:\n");
        printf("a:Auto test key\n");
        printf("c:choose test key\n"); 
        printf("t:control_servo\n");
        printf("s:set posture:\n");
        printf("r:relax arms\n");
        printf("e:exit\n");
        if(EOF == scanf("%c", &choice)) return 0;
        if('a' == choice) {
            auto_test_key();
        }
        else if('c' == choice) {
            chose_key_test();
        }
        else if('e' == choice){
            break;
        }
        else if('t' == choice){
            control_servo();
        }
        else if('s' == choice) {
            set_posture();
        }
        else if('r' == choice){
            relax_arm();
        }
        else{
            return 0;
        }
        
        //打印温度
        print();
        getchar();
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

//读取琴键的位置
void raed_key_pos()
{
    FILE *fpL = NULL, *fpR = NULL;
    int valueL[5], valueR[5];
    int i, j = 0;
    
    fpL = fopen(LKeyPosPath, "r");
    fpR = fopen(RKeyPosPath, "r");
    if((NULL == fpL) || (NULL == fpR)) {
        printf("err:read file %s and %s\n", LKeyPosPath, RKeyPosPath);
        exit(1);
    }
    
    //读取坐标
    while(!feof(fpL)) {
        if(EOF == fscanf(fpL, "%d %d %d %d %d\n", &valueL[0],
            &valueL[1], &valueL[2], &valueL[3], &valueL[4])) {
            printf("err:read_key_pos read %s failed\n", LKeyPosPath);
            exit(1);
        }
        if(EOF == fscanf(fpR, "%d %d %d %d %d\n", &valueR[0],
            &valueR[1], &valueR[2], &valueR[3], &valueR[4])) {
            printf("err:read_key_pos read %s failed\n", RKeyPosPath);
            exit(1);
        }
        
        for(i = 0; i < 5; i++) {
            LKeyPos[j][i] = valueL[i];
            RKeyPos[j][i] = valueR[i];
        }
        j += 1;
    }
    /*
    //显示所读取的值
    //显示左臂shuju
    for(i = 1; i <= 9; i++) {
        printf("(%d) %d %d %d %d %d\n", i, LKeyPos[i][0], LKeyPos[i][1], LKeyPos[i][2], LKeyPos[i][3], LKeyPos[i][4]);
    }
    //显示右臂数据
    for(i = 10; i <= 18; i++) {
        printf("(%d) %d %d %d %d %d\n", i, RKeyPos[i-9][0], RKeyPos[i-9][1], RKeyPos[i-9][2], RKeyPos[i-9][3], RKeyPos[i-9][4]);
    }
    */
    fclose(fpL);
    fclose(fpR);
}

//从A段时平滑的运动到B
void from_A_to_B(int num1, int num2)
{
    int base = 1, start[5], end[5], posK;
    int i, j, path[6][5], diffK[5], speK[5];
    double unit[7];
    
    if((num1 <= 9) && (num2 <= 9)) {
        //完全在左边运动
        base = 6;
        for(i = 0; i < SERVO_NUM/2; i++) {
            start[i] = LKeyPos[num1][i];
            end[i] = LKeyPos[num2][i];
            CurLArm[i] = end[i];
        }
    }
    else if((num1 <= 9) && (num2 >= 10)) {
        //从左运动到右运动
        base = 1;
        for(i = 0; i < SERVO_NUM/2 ; i++) {
            start[i] = CurRArm[i];
            end[i] = RKeyPos[num2 - 9][i];
            CurRArm[i] = end[i];
        }
    }
    else if((num1 >= 10) && (num2 <= 9)) {
        //从右运动到左
        base = 6;
        for(i = 0; i < SERVO_NUM/2; i++) {
            start[i] = CurLArm[i];
            end[i] = LKeyPos[num2][i];
            CurLArm[i] = end[i];
        }
    }
    else if((num1 >= 10) && (num2 >= 10)) {
        //完全在右边运动
        base = 1;
        for(i = 0; i < SERVO_NUM/2; i++) {
            start[i] = RKeyPos[num1 - 9][i];
            end[i] = RKeyPos[num2 - 9][i];
            CurRArm[i] = end[i];
        }
    }
    else 
        exit(1);
        
    //计算刻度之差和速度
    for(i = 0; i < SERVO_NUM/2; i++) {
        diffK[i] = end[i] - start[i];
        speK[i] = (int)(((double)abs(diffK[i]) + 2) * 1.5);
        if(speK[i] >= 1020)
            speK[i] = 1020;
    }
    
    //计算单位刻度
    for(i = 0; i < SERVO_NUM/2; i++)
        unit[i] = (double)diffK[i] / 6.0;
        
    //计算path
    for(i = 1; i <= 6; i++) {
        for(j = 0; j < 5; j++) {
            path[i - 1][j] = unit[j] * i;
        }
    }
    
    //从A运动到B
    //设置速度
    for(i = 0; i < SERVO_NUM/2; i++)
        set_one_servo_word(i+base, Moving_Speed, speK[i]);
    //设置目标位置
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 5; j++) {
            posK = path[i][j] + start[j];
            set_one_servo_word(j+base, Goal_Position, posK);
            delay_us(1 * 1000);
        }
    }
    
    //运动到目标位置
    for(j = 0; j < 5; j++) 
        set_one_servo_word(j + base, Goal_Position, end[j]);
}

//敲打音乐
void hit_and_hit(int num)
{
    int diffK[10] = {3, 8, 12, 16, 20, 24, 28, 31, 34, 39};
    int posK, i;
    
    if(num <= 9) {
        set_one_servo_word(10, Moving_Speed, 359);
        for(i = 0; i < 10; i++) {
            posK = LKeyPos[num][4] + diffK[i];
            set_one_servo_word(10, Goal_Position, posK);
            delay_us(5 * 1000);
        }
        set_one_servo_word(10, Moving_Speed, 559);
        for(i = 8; i >= 0; i--) {
            posK = LKeyPos[num][4] + diffK[i];
            set_one_servo_word(10, Goal_Position, posK);
            delay_us(5 * 1000);
        }
    }
    
    if(num >= 10) {
        set_one_servo_word(5, Moving_Speed, 359);
        for(i = 0; i < 10; i++) {
            posK = RKeyPos[num-9][4] - diffK[i];
            set_one_servo_word(5, Goal_Position, posK);
            delay_us(5 * 1000);
        }
        set_one_servo_word(5, Moving_Speed, 559);
        for(i = 8; i >= 0; i--) {
            posK = RKeyPos[num-9][4] - diffK[i];
            set_one_servo_word(5, Goal_Position, posK);
            delay_us(5*1000);
        }
    }
}

//重新上刚度
void re_enable()
{
    int i, value[2][10];
    int flag;

    while(1) {
        //读取舵机当前值
        for(i = 0; i < SERVO_NUM; i++)
            value[0][i] = get_one_servo_word(i+1, Present_Position);
            
        //间隔5s
        delay_us(5 * 1000 * 1000);
        
        //读取舵机当前刻度
        for(i = 0; i < SERVO_NUM; i++)
            value[0][i] = get_one_servo_word(i+1, Present_Position);
            
        //如两次刻度值之差在10K内则上刚度
        flag = 0;
        for(i = 0; i < SERVO_NUM; i++) {
            if(abs(value[0][i] - value[1][i]) <= 10)
                flag += 1;
        }
        if(10 == flag) {
            for(i = 0; i < SERVO_NUM; i++) {
                set_one_servo_byte(i+1, Torque_Enable, 1);
                
                //现实刻度
                printf("(%d) %d\n", i+1, value[1][i]);
            }
            printf("\n");
        }
    }
}

//释放刚度
void relax()
{
    int reload[2], i;
    
    //获取5和10号舵机的relaod
    reload[0] = get_one_servo_word(5, 40);
    reload[1] = get_one_servo_word(10, 40);
    
    while(1) {
        if(reload[0]%1024 >= 80) {
            for(i = 1; i <= 5; i++)
                set_one_servo_byte(i, Torque_Enable, 0);
            break;
        }
        else if(reload[1]%1024 >= 80) {
            for(i = 6; i <= 10; i++)
                set_one_servo_byte(i, Torque_Enable, 0);
            break;
        }
        else{
            reload[0] = get_one_servo_word(5, 40);
            reload[1] = get_one_servo_word(10, 40);
            delay_us(1000*1000);
            printf("\rid(5) relaod(%d); id(10) relaod(%d)", reload[0], reload[1]); fflush(stdout);
        }
    }
    printf("\n");
}

//系统初始化
void initial_sys(int device, int bandnum)
{
    int i;
    
    if(dxl_initialize(device, bandnum) == 0) {
        printf("Failed to open USB2Dynamixel!\n");
        exit(1);
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
    for(i = 0; i < SERVO_NUM; i++) 
        set_one_servo_byte(i +1, Torque_Enable, 1);
    
    //读取琴键位置文件
    raed_key_pos();
}

//设置初始化位置
void set_default_pos()
{
    int i, result;
    
    //设置速度
    for(i = 0; i < SERVO_NUM; i++) {
        result = set_one_servo_word(i+1, Moving_Speed, 60);
        if(0 == result) {
            printf("err:default_pos set speed failed...\n");
            exit(1);
        }
    }
    
    //设置目标位置
    set_one_servo_word(5, Goal_Position, RKeyPos[5][4]);
    delay_us(200*1000); //200ms
    set_one_servo_word(10, Goal_Position, LKeyPos[5][4]);
    delay_us(200*1000); //200ms
    for(i = 0; i < 4; i++) {
        set_one_servo_word(i + 1, Goal_Position, RKeyPos[5][i]);
        set_one_servo_word(i + 6, Goal_Position, LKeyPos[5][i]);
    }
    
    //等待运动停止
    wait_arm_stop_exten('L', 10);
    wait_arm_stop_exten('R', 10);
    
    //更新当前姿态
    for(i = 0; i < 5; i++) {
        CurLArm[i] = LKeyPos[5][i];
        CurRArm[i] = RKeyPos[5][i];
    }
}

//自动测试每个按键位置
void auto_test_key()
{
    int start, end;
    
    //运动到第一个位置
    start = 1;
    set_arm_word('L', Goal_Position, LKeyPos[1]);
    wait_arm_stop_exten('L', 10);
    getchar();
    hit_and_hit(1);
    delay_us(2000 * 1000);
    
    for(end = 2; end <= 18; end++) {
        //移动
        from_A_to_B(start, end);
        delay_us(1000*1000);
        //getchar();
        //敲击
        hit_and_hit(end);
        //更新
        start = end;
    }
}

//手动选择琴键
void chose_key_test()
{
    int end;
    
    //选择要测试的琴键
    printf("please input the key you want to test:\n");
    if(EOF == scanf("%d", &end)) return;
    
    //运动到目标位置
    if(end <= 9) {
        set_arm_word('L', Goal_Position, LKeyPos[end]);
        wait_arm_stop_exten('L', 10);
    }
    else {
        set_arm_word('R', Goal_Position, RKeyPos[end - 9]);
        wait_arm_stop_exten('R', 10);
    }
    
    //敲击
    hit_and_hit(end);
}

//设置姿态
void set_posture()
{
    int num = 0;
    
    printf("welcome to set_posture...\n");
    getchar();
    
    while(1) {
        //检测机械臂是否静止
        relax();
        
        //检测机械臂是否在运动
        re_enable();
        
        //延时
        delay_us(2*1000*1000);
        printf("num:%d\n", num++);
    }
    
}

//控制机械臂的姿态
int control_servo()
{
    int value, reload;
    int valK[5], i, id;
    char choice;

    printf("welcome to control arms posture...\n");
    while(1){
        printf("please choose control mode:q, exit; s, sigle; m, arms\n"); fflush(stdout);
        if(0 == scanf("%c", &choice)) return 0;
        if('q' == choice) {
            printf("being exiting ...\n");
            break;
        }
        else if('s' == choice) {
            printf("please input servo id:"); fflush(stdout);
            if(0 == scanf("%d", &id)) return 0;
            value = get_one_servo_word(id, Present_Position);
            reload = get_one_servo_word(id, 40);
            printf("%d servo pre-position %d Load %d\n", id, value, reload%1024);
            printf("please input servo goal-position:");
            if(0 == scanf("%d", &value)) return 0;
            set_one_servo_word(id, Goal_Position, value);
            wait_for_one_servo(id);
        }
        else if('m' == choice) {
            printf("please choose 'L' or 'R'...");
            if(0 == scanf("%c", &choice)) return 0;
            if('L' == choice){
                //获取当前值
                for(i = 0; i < SERVO_NUM/2; i++) {
                    valK[i] = get_one_servo_word(i+6, Present_Position);
                    printf("servo (%d) pre-pos %d\n", i+6, valK[i]);
                }
                printf("please inout servo goal-pos:");
                if(0 == scanf("%d %d %d %d %d", &valK[0], &valK[1], &valK[2], &valK[3], &valK[4])) return 0;
                for(i = 0; i < SERVO_NUM/2; i++)
                    set_one_servo_word(i + 6, Goal_Position, valK[i]);
            }
            else{
                //获取当前的值
                for(i = 0; i < SERVO_NUM/2; i++) {
                    valK[i] = get_one_servo_word(i+1, Present_Position);
                    printf("servo (%d) pre-pos %d\n", i+1, valK[i]);
                }
                printf("please input servo goal-pos:");
                if(0 == scanf("%d %d %d %d %d", &valK[0], &valK[1], &valK[2], &valK[3], &valK[4])) return 0;
                for(i = 0; i < SERVO_NUM/2; i++)
                    set_one_servo_word(i + 6, Goal_Position, valK[i]);
            }
        }
        else{
        
        }
    }
    
    return 1;
}

//放松手臂
void relax_arm()
{
    int i;
    
    for(i = 1; i <= SERVO_NUM; i++)
        set_one_servo_byte(i, Torque_Enable, 0);
}

//打印此时温度
void print()
{
    int i, temprature;
    
    for(i = 1; i <= SERVO_NUM; i++) {
        temprature = get_one_servo_byte(i, 43);
        printf("the id(%d) temprature(%d)\n", i, temprature);
    }
}
