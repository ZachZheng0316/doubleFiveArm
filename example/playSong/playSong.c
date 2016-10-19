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

//默认设置
#define SERVO_NUM (10)
#define NOARMS (2)

static int DEVICE = 0;
static int BAUDNUM = 1;
static int CurLArm[5]; //左臂当前的位置
static int CurRArm[5]; //右臂当前的位置
static long tm1 = 590 - 300;
static long tm2 = 1650 + 600;
static int LKeyPos[10][5];
static int RKeyPos[10][5];
static char LKeyPosPath[] = "LKeyPos.txt";
static char RKeyPosPath[] = "RKeyPos.txt";
static char SongPath[] = "smallW.txt";
static char TrajPath[] = "trajectory.txt";

//子功能函数
int set_arm_byte(char dir, int address, int *value);
int set_arm_word(char dir, int address, int *value);
int wait_arm_stop_exten(char dir, int exten); //等待运动停止
void read_key_pos(); //读取琴键位置
void tra_key_to_arm(int keyNum, char *dir, int *armK); //由琴键序列数得到arm姿态
int cal_spe(int k1, int k2, long tm); //计算速度
void tra_song_to_trac(char *path1, char *path2); //把曲目转换成路径
void exc_song(char *songPath); //执行音乐路径
void hit_and_hit(char dir, int value5); //敲打音乐
void from_A_to_B(int num1, int num2); //从A短时平滑的运动到B
void exc_song_test(char *songPath); //

//架构函数
int initial_sys(int device, int bandnum); //系统初始化
int default_pos();                        //初始化位置
int relax_arm(char dir);                  //放松手臂
void print_temprature(char dir);          //打印此时的温度

int main()
{
    //系统初始化
    if(1 != initial_sys(DEVICE, BAUDNUM)) {
        printf("err:initial failed ...\n");
        getchar();
        return 0;
    }
    
    //读取琴键位置值
    read_key_pos();
    
    //运动到初始位置
    if(1 != default_pos()) {
        printf("err:default_pos ...\n");
        getchar();
        return 0;
    }
    
    /*
    //产生运动数据
    tra_song_to_trac(SongPath, TrajPath);
    
    printf("start excute ...\n");
    getchar();
    //执行运动数据
    exc_song(TrajPath);
    */
    printf("start moving smooth ...\n");
    getchar();
    //from_A_to_B(14, 9); //从A短时平滑的运动到B
    exc_song_test(SongPath);
    
    printf("relax ...\n");
    getchar();
    relax_arm('L');
    relax_arm('R');
    print_temprature('L');
    print_temprature('R');
    
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

//读取坐标
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
    /*
    //显示左臂数据
    for(i = 0; i < 10; i++) {
        printf("%d  ", i);
        for(j = 0; j < 5; j++)
            printf("%d ", LKeyPos[i][j]);
        printf("\n");
    }
    printf("\n");
    //显示右臂数据
    for(i = 0; i < 10; i++) {
        printf("%d  ", i);
        for(j = 0; j < 5; j++) 
            printf("%d ", RKeyPos[i][j]);
        printf("\n");
    }*/
    
    fclose(fpL);
    fclose(fpR);
}

//由琴键序列数得到arm姿态
void tra_key_to_arm(int keyNum, char *dir, int *armK)
{
    int i;
    
    //左臂
    if((1 <= keyNum) && (keyNum <= 9)) {
        for(i = 0; i < 5; i++) 
            armK[i] = LKeyPos[keyNum][i];
        *dir = 'L';
        
        return;
    }
    
    //右臂
    if((10 <= keyNum) && (keyNum <= 18)) {
        for(i = 0; i < 5; i++) 
            armK[i] = RKeyPos[keyNum - 9][i];
        *dir = 'R';
        return;
    }
    
    //如果遇到是0
    if(0 == keyNum) {
        for(i = 0; i < 5; i++) 
            armK[i] = 1;
        *dir = 'X';
        return;
    }
    
    exit(1);
}

//计算速度
int cal_spe(int k1, int k2, long tm)
{
    double dt, angSpe, rate = 0.3;
    int diffK, speK;
    
    tm = (long)((double)tm * rate);
    
    diffK = abs(k1 - k2);
    dt = (double)tm / 1000.0;
    angSpe = diffK * PositionUnit; //计算角度
    angSpe = angSpe / dt;          //计算角速度
    angSpe = angSpe / 6;           //计算rpm
    speK = (int)(angSpe / SpeedUnit + 1.35);
    
    return speK;
}

//把曲目信息转化为
void tra_song_to_trac(char *path1, char *path2)
{
    FILE *fpR = NULL, *fpW = NULL;
    char flag, dir;
    int keyNum, valueK[5], speK[5];
    int i;
    long tm;
    
    //打开文件是否正确
    fpR = fopen(path1, "r");
    fpW = fopen(path2, "w");
    if((NULL == fpR) || (NULL == fpW)) {
        printf("err:read file %s and %s\n", path1, path2);
        exit(1);
    }
    
    //读取文件数据
    while(!feof(fpR)) {
        if(EOF == fscanf(fpR, "%c %d\n", &flag, &keyNum)) {
            printf("err:read %s file failed\n", path1);
            exit(1);
        }
        //获取暂留时间
        if('A' == flag)
            tm = tm2;
        if('a' == flag)
            tm = tm1;
        //获取琴键值
        tra_key_to_arm(keyNum, &dir, valueK);
        
        if('L' == dir) {
            //获得左臂姿态速度
            for(i = 0; i < 5; i++) 
                speK[i] = cal_spe(CurLArm[i], valueK[i], tm);
            
            //更新当前姿态
            for(i = 0; i < 5; i++) 
                CurLArm[i] = valueK[i];
        } 
        
        if('R' == dir) {
            //获得右臂姿态速度
            for(i = 0; i < 5; i++)
                speK[i] = cal_spe(CurRArm[i], valueK[i], tm);
            
            //更新当前姿态
            for(i = 0; i < 5; i++)
                CurRArm[i] = valueK[i];
        }
        
        if('X' == dir) {
            //获取左臂数据    
            for(i = 0; i < 5; i++)
                speK[i] = 1;
            
            //不更新姿态
        }
        
        //存储
        fprintf(fpW, "(%ld) (%c) (%d %d %d %d %d) (%d %d %d %d %d)\n", 
                      tm, dir, speK[0], speK[1], speK[2], speK[3], speK[4], 
                      valueK[0], valueK[1], valueK[2], valueK[3], valueK[4]);
    }
    
    fclose(fpR);
    fclose(fpW);
}

//执行路径信息
void exc_song(char *songPath)
{
    FILE *fpR = NULL;
    long tm;
    char dir;
    int speK[5], valueK[5];
    
    fpR = fopen(songPath, "r");
    if((NULL == fpR)) {
        printf("err:read file %s\n", songPath);
        exit(1);
    }
    
    while(!feof(fpR)) {
        if(EOF == fscanf(fpR, "(%ld) (%c) (%d %d %d %d %d) (%d %d %d %d %d)\n", 
                        &tm, &dir, &speK[0], &speK[1], 
                        &speK[2], &speK[3], &speK[4], 
                        &valueK[0], &valueK[1], &valueK[2], 
                        &valueK[3], &valueK[4])) {
            printf("err:read %s file failed\n", songPath);
            exit(1);
        }
        
        tm = tm / 5; //3
        
        if('L' == dir) {
            //运动到目标位置
            set_arm_word(dir, Moving_Speed, speK);
            set_arm_word(dir, Goal_Position, valueK);
            delay_us(tm * 1000);
            wait_arm_stop_exten(dir, 8);
            
            //敲击琴键
            hit_and_hit(dir, valueK[4]);
        }
        
        if('R' == dir) {
            //运动到目标位置
            set_arm_word(dir, Moving_Speed, speK);
            set_arm_word(dir, Goal_Position, valueK);
            delay_us(tm * 1000);
            wait_arm_stop_exten(dir, 8);
            
            //敲击琴键
            hit_and_hit(dir, valueK[4]);
        }
        
        if('X' == dir) 
            delay_us(tm * 1000);
    }
    
    fclose(fpR);
}

//敲打音乐
void hit_and_hit(char dir, int value5)
{
    int diffK[10] = {3, 8, 12, 16, 20, 24, 28, 31, 34, 39};
    int posK, i;
    
    if('L' == dir) {
        set_one_servo_word(10, Moving_Speed, 359);
        for(i = 0; i < 10; i++) {
            posK = value5 + diffK[i];
            set_one_servo_word(10, Goal_Position, posK);
            delay_us(5 * 1000); //50ms
            //printf("posK:%d\n", posK);
        }
        set_one_servo_word(10, Moving_Speed, 559);
        for(i = 8; i >= 0; i--) {
            posK = value5 + diffK[i];
            set_one_servo_word(10, Goal_Position, posK);
            delay_us(5 * 1000);
            //printf("posK:%d\n", posK);
        }
    }
    
    if('R' == dir) {
        set_one_servo_word(5, Moving_Speed, 359);
        for(i = 0; i < 10; i++) { 
            posK = value5 - diffK[i];
            set_one_servo_word(5, Goal_Position, posK);
            delay_us(5 * 1000); //50ms
            //printf("posK:%d\n", posK);
        }
        set_one_servo_word(5, Moving_Speed, 559);
        for(i = 8; i >= 0; i--) {
            posK = value5 - diffK[i];
            set_one_servo_word(5, Goal_Position, posK);
            delay_us(5 * 1000);
            //printf("posK:%d\n", posK);
        }
    }
}

//从A短时平滑的运动到B
void from_A_to_B(int num1, int num2)
{
    int base = 1, start[5], end[5], posK;
    int i, j, path[6][5], diffK[5], speK[5];
    double unit[7];
    
    if((num1 <= 9) && (num2 <= 9)) {
        //完全在左边运动
        base = 6;
        
        for(i = 0; i < 5; i++) {
            start[i] = LKeyPos[num1][i];
            end[i] = LKeyPos[num2][i];
            //更新当前坐标
            CurLArm[i] = end[i];
        }
    }
    else if((num1 <= 9) && (num2 >= 10)){
        //从左运动到右运动
        base = 1;
        
        for(i = 0; i < 5; i++) {
            start[i] = CurRArm[i];
            end[i] = RKeyPos[num2 - 9][i];
            //更新当前坐标
            CurRArm[i] = end[i];
        }
    }
    else if((num1 >= 10) && (num2 <= 9)) {
        //从右运动到左运动
        base = 6;
        
        for(i = 0; i < 5; i++) {
            start[i] = CurLArm[i];
            end[i] = LKeyPos[num2][i];
            //更新当前坐标
            CurLArm[i] = end[i];
        }
    }
    else if((num1 >= 10) && (num2 >= 10)) {
        //完全在右边运动
        base = 1;
        
        for(i = 0; i < 5; i++){
            start[i] = RKeyPos[num1 - 9][i];
            end[i] = RKeyPos[num2 - 9][i];
            //更新当前坐标
            CurRArm[i] = end[i];
        }
    }
    else
        exit(1);
        
    //计算刻度之差和速度
    for(i = 0; i < 5; i++) {
        diffK[i] = end[i] - start[i];
        speK[i] = (int)((double)abs(diffK[i] + 2) * 2.0);
    }
    
    //计算单位刻度
    for(i = 0; i < 5; i++)
        unit[i] = (double)diffK[i] / 6.0;
        
    //计算path
    for(i = 1; i <= 6; i++) {
        for(j = 0; j < 5; j++) {
            path[i - 1][j] = unit[j] * i;
        }
    }
    
    //从A运动到B
    //设置速度
    for(i = 0; i < 5; i++) 
        set_one_servo_word(i + base, Moving_Speed, speK[i]);
    //设置目标位置
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 5; j++) {
            posK = path[i][j] + start[j];
            set_one_servo_word(j + base, Goal_Position, posK);
            delay_us((long)(2.5 * 1000));
        }
    }
    
    //运动到目标位置
    for(j = 0; j < 5; j++) 
        set_one_servo_word(j + base, Goal_Position, end[j]);
}

//
void exc_song_test(char *songPath)
{
    FILE *fpR = NULL;
    int start, end;
    double delay1, delay2;
    long delay;
    
    fpR = fopen(songPath, "r");
    if(NULL == fpR){
        printf("open %s failed\n", songPath);
        exit(1);
    }
    
    //读取第一个位置
    if(EOF == fscanf(fpR, "%d %lf\n", &start, &delay1)){
        printf("read file %s failed\n", songPath);
        exit(1);
    }
    if(start <= 9) {
        set_arm_word('L', Goal_Position, LKeyPos[start]);
        wait_arm_stop_exten('L', 10);
        hit_and_hit('L', LKeyPos[start][4]);
    }
    else{
        set_arm_word('R', Goal_Position, RKeyPos[start - 9]);
        wait_arm_stop_exten('R', 10);
        hit_and_hit('R', RKeyPos[start - 9][4]);
    }
    
    
    while(!feof(fpR)) {
        if(EOF == fscanf(fpR, "%d %lf\n", &end, &delay2)){
            printf("read fiel %s faield\n", songPath);
            exit(1);
        }
        
        //判断读取数据是否为0
        if(0 == end) {
            delay = (long)(delay1 * 80.0 / 0.25);
            delay_us(delay * 1000);
        }
        else{
            //读取的数据不为0
            from_A_to_B(start, end);
            
            //延时delay1
            delay = (long)(delay1 * 80.0 / 0.25);
            delay_us(delay * 1000);
            
            //敲击
            if(end <= 9)
                hit_and_hit('L', LKeyPos[end][4]);
            else
                hit_and_hit('R', RKeyPos[end - 9][4]);
            
            //更新当前值
            start = end;
            delay1 = delay2;
        }
        
    }
    
    fclose(fpR);
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
#if (NOARMS == 1)
    for(i = 0; i < SERVO_NUM; i++) {
        set_one_servo_byte(i+1, CW_Slope, 64);
        set_one_servo_byte(i+1, CCW_Slope, 64);
    }
    set_one_servo_byte(2, CW_Slope, 32);
    set_one_servo_byte(2, CCW_Slope, 32);
    set_one_servo_byte(7, CW_Slope, 32);
    set_one_servo_byte(7, CCW_Slope, 32);
#elif (NOARMS == 2)
    for(i = 0; i < SERVO_NUM; i++) {
        set_one_servo_byte(i+1, CW_Slope, 32);
        set_one_servo_byte(i+1, CCW_Slope, 32);
    }
#endif
        
    //激活
    for(i = 0; i < SERVO_NUM; i++) 
        set_one_servo_byte(i +1, Torque_Enable, 1);
    
    return 1;
}

//初始化位置
int default_pos()
{
    int i, result;
    
    //设置速度
    for(i = 0; i < SERVO_NUM; i++) {
        result = set_one_servo_word(i+1, Moving_Speed, 60);
        if(0 == result) {
            printf("err:default_pos set speed failed...\n");
            return 0;
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
