#pragma once
#include "MapFunction.h"
#include "HungarianAlgorithm.h"

class RobotSchedule {
public:
    RobotSchedule();

    // 机器人调度
    void schedule(MapFunction& mapFunction, GlobalData& globalData);

private:
    void preProcess(MapFunction &mapFunction, GlobalData& globalData);
    // 为机器人设定目标
    void optRobots(MapFunction& mapFunction, GlobalData& globalData);
    // 移动机器人
    void moveRobots(MapFunction& mapFunction, GlobalData& globalData);

    //由于场上有非常多的机器人，没办法预测别人机器人的动向，所以决赛不使用方向锁
    //每个机器人只检查是否会与id比自身大的机器人发生碰撞
    map<pair<int, int>, vector<int>> m_robotLock;


    // 未装满货物的机器人id
    set<int> robotsTakeGood;
    // 装满货物的机器人id
    set<int> robotsGoBerth;
    // 通过berth引导，去找高价值货物的机器人
    set<int> robotsFindGood;

    set<int> goodsTaken;

    int robotSendWorth;
};