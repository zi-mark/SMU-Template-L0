/*
 *
 * 你好，我是SMU-Template的编写者Mark Ma，很高兴您使用SMU-Template作为您VEX之旅的模板程序！
 *
 * 请您注意，此模板为L0！！L0！！L0级别难度模板
 *   **L0**  |  L1  |  L2  |  L3
 * 请根据队伍自身水平选择相应模板使用
 * L0级别模板适合初学者使用，其要求学生熟练掌握以下内容：
 *   - 基础的VEX编程语法
 *   - 基础的VEX编程逻辑
 *
 * 请您注意，Student Center Policy是VEX项目中非常核心的价值理念和行为准则
 * 也就意味着在任何情况下教练都不应直接帮助学生进行编程操作
 * 不可否认，模板库的出现极大的减少了重复工作，增加了编程效率
 * 使用与队伍水平匹配的模板进行编程符合SCP的要求
 * 但请您注意，模板库并不代表教练可以直接帮助学生编程
 * 请您在使用模板库时，务必遵守Student Center Policy
 * 保证程序库内容水平与队伍编程手水平相当
 * 以免造成不必要的麻烦与冲突
 * 
 * 本库为本人近年来参赛经验总结而来，由于本人不是专职的编程手，代码方面有诸多不合理的地方，请大家谅解
 * 如您需要更严谨，更丰富的内容请寻找L1，L2，L3级别的SMU-Template，谢谢!
 *
 * Copyright (c) 2025 SMU Robotics Team
 */

#include "vex.h"

using namespace vex;

//Robot Configuration
brain Brain;
controller Con;
competition Com;

//motor 马达名字(端口，变速箱比例，是否反转 0不反转1反转);
motor R1(PORT1, ratio6_1, 0);
motor R2(PORT2, ratio6_1, 0);
motor R3(PORT5, ratio6_1, 0);

motor L1(PORT10, ratio6_1, 1);
motor L2(PORT9, ratio6_1, 1);
motor L3(PORT6, ratio6_1, 1);

//轮子半径和齿轮比 马达齿/轮子齿
double WheelRadius = 2;
double ChasisRatio = 35.0/60.0;
double ChasisWidth = 7.5;

motor LiftL(PORT19, ratio36_1, 1);
motor LiftR(PORT12, ratio36_1, 0);
// motor Lift(PORT10, ratio36_1, 0);


motor Suck(PORT20, ratio6_1, 1);
motor Suck2(PORT11, ratio6_1, 0);

//惯性传感器(端口);
inertial GR(PORT21);
optical CLSensor(PORT20);

//电磁阀(三线接口);
digital_out Hook(Brain.ThreeWirePort.A);
//

//底盘函数
void SpinLR(double lv, double rv, vex::velocityUnits vu = velocityUnits::pct){
    L1.spin(fwd, lv, vu);
    L2.spin(fwd, lv, vu);
    L3.spin(fwd, lv, vu);
    R1.spin(fwd, rv, vu);
    R2.spin(fwd, rv, vu);
    R3.spin(fwd, rv, vu);
}

void Stop(vex::brakeType mode = brakeType::brake){
    L1.stop(mode);
    L2.stop(mode);
    L3.stop(mode);
    R1.stop(mode);
    R2.stop(mode);
    R3.stop(mode);
}

//内置编码器直走
void Go(double target, double v, velocityUnits vu){
    target = target * 360 / (2 * 3.1415926 * WheelRadius * ChasisRatio);
    L1.spinFor(target, deg, v, vu, 0);
    L2.spinFor(target, deg, v, vu, 0);
    L3.spinFor(target, deg, v, vu, 0); 
    R1.spinFor(target, deg, v, vu, 0);
    R2.spinFor(target, deg, v, vu, 0);
    R3.spinFor(target, deg, v, vu, 0);
    bool finish = 1;
    while(finish){
        finish *= !L1.isDone() * !L2.isDone() * !L3.isDone();
        finish *= !R1.isDone() * !R2.isDone() * !R3.isDone();
    }
    Stop(hold);
}

//内置编码器转相对角度
void TurnFor(double target, double v, velocityUnits vu){
    target = target * ChasisWidth / (2 * WheelRadius * ChasisRatio);
    L1.spinFor(target, deg, v, vu, false);
    L2.spinFor(target, deg, v, vu, false);
    L3.spinFor(target, deg, v, vu, false);
    R1.spinFor(-target, deg, v, vu, false);
    R2.spinFor(-target, deg, v, vu, false);
    R3.spinFor(-target, deg, v, vu, false);
    while(1){
        bool done = true;
        if(!L1.isDone() || !R1.isDone()){
            done = false;
        }
        if(!L2.isDone() || !R2.isDone()){
            done = false;
        }
        if(!L3.isDone() || !R3.isDone()){
            done = false;
        }
        if(done) break;
        task::sleep(1);
    }
    Stop(hold);
}
//

//Initialization
void Init(){
        
        GR.calibrate();
        
        while(GR.isCalibrating()) continue;
        
    }
//

//Autonomous
void Auto(){

}
//

//drivercontrol
void DC(){
    int lv, rv;
    Stop(coast);
    Brain.Screen.clearScreen(black);
    while(1){
        //单杆
        lv = Con.Axis3.position() + Con.Axis4.position();
        rv = Con.Axis3.position() - Con.Axis4.position();
        //双杆
        // lv = Con.Axis3.position() + Con.Axis1.position();
        // rv = Con.Axis3.position() - Con.Axis1.position();


        if(abs(lv) < 5) lv = 0;
        if(abs(rv) < 5) rv = 0;
        SpinLR(lv, rv);
        
    }
}
//

//main
timer Auto_T;
int main(){
    Brain.Screen.clearScreen();
    Init();
    if(Com.isCompetitionSwitch() || Com.isFieldControl()){
        Com.drivercontrol(DC);
        Com.autonomous(Auto);
        Con.Screen.print("Competition ");
    }
    else{
        Con.Screen.print("Not Competition ");
        while(!Con.ButtonA.pressing()) continue;
        Auto_T.reset();
        Auto();
        Con.Screen.newLine();
        Con.Screen.print(Auto_T.value());
        DC();
    }
    
    
 while(1) wait(10,msec);
}
//
