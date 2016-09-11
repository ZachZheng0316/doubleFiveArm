#include <stdio.h>
#include <stdlib.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "AX12A.h"
#include "dmath.h"
#include "serialCommuni.h"

//设置默认值
#define SERVO_NUM (10)
static int DEVICE = 0;
static int BAUDNUM = 1;
static int CurLArm[5];
static int CurRArm[5];
static int CurKeyNum;
static int LKeyPos[10][5]; 
static int RKeyPos[10][5];
static char LKeyPosPath[] = "LKeyPos.txt";
static char RKeyPosPath[] = "RKeyPos.txt";

//子功能函数
void read_key_pos();                         //读取琴键的位置
int set_arm_byte(char dir, int address, int *value);
int set_arm_word(char dir, int address, int *value);
int wait_arm_stop_exten(char dir, int exten);//等待运动停止
void from_A_to_B(int num1, int num2);        //从A段时平滑的运动到B
void hit_and_hit(int num);                   //敲打音乐
void play_misic(int musicNum);               //演奏曲目
void hit_key(int keyNum);                    //即兴演奏
void play_temp_music(char *keyNum); //演奏临时曲目

//框架程序
void initial_sys(int device, int bandnum);  //系统初始化
void set_default_pos();                     //设置初始化位置
void receive_instruct(unsigned char *instruct);
void execute_instruct(unsigned char *instruct);
void finish_instruct(unsigned char *feedback);
void relax_arm();                           //放松手臂
void print();                               //打印此时温度
void finish_sys();                          //关闭系统

int main()
{
    unsigned char packet[60] = {0, };  //接受指令的数据包
    
    initial_sys(DEVICE, BAUDNUM);
    
    do{
        receive_instruct(packet);
        
        execute_instruct(packet);
    }while(1);
    
    finish_sys();
    
    return 1;
}

//主框架程序
//系统初始化
void initial_sys(int device, int bandnum)
{
    int i;
    
    //打开串口
    if(1 != serial_open(5, 1200)) {
        printf("doubelFiveArms:initial_sys:serial_open failed\n");
        exit(0);
    }
    
    //打开USB2Dynamixel
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
    read_key_pos();
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

void receive_instruct(unsigned char *instruct)
{
    int flag;
    
    while(1) {
        //1.获取字符
        flag = receiveMessage(instruct, 60);
        if(-1 == flag) {
            printf("\rreceive data failed!");
            fflush(stdout);
        }
        else if(0 == flag) {
            printf("\ncontinue receiving data ...");
            fflush(stdout);
        }
        else if(flag > 0) {
            printf("print data ...");
            fflush(stdout);
            if(flag >= 59)
                instruct[59] = '\0';
            else
                instruct[flag] = '\0';
            printf("the receiving data (%s)\n", instruct);
            fflush(stdout);
            break;
        }
        else{
        }
    }
}

void execute_instruct(unsigned char *instruct)
{
    char temp[60] = {0, };
    int num;
    
    if(('c' == instruct[0]) || ('h' == instruct[0]) || ('p' == instruct[0])){
        //位置初始化
        set_default_pos(); //位置初始化
        
        if('c' == instruct[0]) {
            //解析信息
            strcpy(temp, (char *)&instruct[1]);
            num = atoi(temp);
            
            //执行弹奏曲子
            play_misic(num);
            
            //反馈信息
            finish_instruct((unsigned char *)"oover");
        }
        else if('h' == instruct[0]){
            //解析信息
            strcpy(temp, (char *)&instruct[1]);
            num = atoi(temp);
            
            //敲击琴键
            hit_key(num);
            
            //反馈信息
            finish_instruct((unsigned char *)"ook");
        }
        else if('p' == instruct[0]){
            //解析信息
            strcpy(temp, (char *)&instruct[1]);
            
            //执行信息
            play_temp_music(temp);
            
            //反馈信息
            finish_instruct((unsigned char *)"oover");
        }
        else{
        }
        
        //松掉刚度
        relax_arm();
    }
    
    //打印温度
    print();
}

void finish_instruct(unsigned char *feedback)
{
    int feedNum, realLen;

	feedNum = strlen((char *)feedback);
	
	while(1) {
        realLen = sendMessage(feedback, feedNum);
		if(realLen != feedNum) {
			printf("failed send data\r"); fflush(stdout);
		}
		else {
			printf("success send data:%s\n", feedback); fflush(stdout);
			break;
		}	
     }
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

//关闭系统
void finish_sys()
{
    serial_close();
    dxl_terminate();
}

//子功能程序
//读取琴键的位置
void read_key_pos()
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
    
    fclose(fpL);
    fclose(fpR);
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
        speK[i] = (int)(((double)abs(diffK[i]) + 2) * 2.0);
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
    //运动到目标位置
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 5; j++) {
            posK = path[i][j] + start[j];
            set_one_servo_word(j+base, Goal_Position, posK);
            delay_us(2.5 * 1000);
        }
    }
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

//演奏曲目
void play_misic(int musicNum)
{
    FILE *fpR = NULL;
    char path[20] = {0, }, keyChar[5] = {0, };
    int start, end;
    double delay1, delay2;
    long delay;
    
    sprintf(path, "songs/%d.txt", musicNum);
    fpR = fopen(path, "r");
    if(NULL == fpR) {
        printf("open %s failed\n", path);
        exit(1);
    }
    
    //读取第一个位置
    if(EOF == fscanf(fpR, "%d %lf\n", &start, &delay1)) {
        printf("read file %s failed\n", path);
        exit(1);
    }
    if(start <= 9) {
        set_arm_word('L', Goal_Position, LKeyPos[start]);
        wait_arm_stop_exten('L', 10);
    }
    else{
        set_arm_word('R', Goal_Position, LKeyPos[start]);
        wait_arm_stop_exten('R', 10);
    }
    CurKeyNum = start;
    
    //发送准备信号
    finish_instruct((unsigned char *)"ook");
    
    //完成打击动作
    hit_and_hit(start);
    
    //发送打击键序
    sprintf(keyChar, "%.3d", start);
    finish_instruct((unsigned char*)keyChar);
    
    while(!feof(fpR)){
        if(EOF == fscanf(fpR, "%d %lf\n", &end, &delay2)){
            printf("read fiel %s faield\n", path);
            exit(1);
        }
        else{
            //读数据不为0
            from_A_to_B(start, end);
            
            //延时delay1
            delay = (long)(delay1 * 80.0 / 0.25);
            delay_us(delay * 1000);
            
            //完成打击动作
            hit_and_hit(end);
            
            //发送打击键序
            sprintf(keyChar, "%.3d", end);
            finish_instruct((unsigned char *)keyChar);
            
            //更新当前的值
            start = end;
            delay1 = delay2;
            CurKeyNum = end;
        }
    }
    
    //回到初始位置
    from_A_to_B(CurKeyNum, 14);
    CurKeyNum = 14;
    from_A_to_B(CurKeyNum, 5);
    CurKeyNum = 5;
    
    fclose(fpR);
}

//即兴演奏
void hit_key(int keyNum)
{
    
    //运动到目标键序
    from_A_to_B(CurKeyNum, keyNum);
    
    //确认停稳
    if(keyNum <= 9)
        wait_arm_stop_exten('L', 10);
    else
        wait_arm_stop_exten('R', 10);
    
    //打击琴键
    hit_and_hit(keyNum);
    
    //更新当前键序
    CurKeyNum = keyNum;
}

//演奏临时曲目
void play_temp_music(char *keyNum)
{
    int amount, i, j, num;
    char keyChar[4] = {0,};
    
    amount = strlen(keyNum);
    
    for(i = 0; i < amount; i += 3) {
        for(j = 0; j < 3; j++) 
            keyChar[j] = keyNum[i+j];
        //字符转化为数字
        num = atoi(keyChar);

        //移动到目标位置
        hit_key(num);
        
        //返回敲击的键序
        finish_instruct((unsigned char *)keyChar);
        
        //延时一定的时间
        delay_us(1 * 1000 * 1000);
    }
    
    //回到初始位置
    from_A_to_B(CurKeyNum, 14);
    CurKeyNum = 14;
    from_A_to_B(CurKeyNum, 5);
    CurKeyNum = 5;
}
