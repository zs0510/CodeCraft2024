#pragma once

#if defined(_WIN32) || defined(__APPLE__)
// 本地运行(Windows / MacOS)
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <stack>
#include <algorithm>
#include <sys/time.h>
#else
// 服务器运行(Linux)
#include <bits/stdc++.h>
#endif

using namespace std;

const int INF = 0x3fffffff;// 无穷大的正整数

const int szOfMap = 800;

// 货物存在的帧率
const int frameOfGoodExist = 1000;

// 最大的帧 ID
const int maxFrameID = 20000;
const int epsilonFrame = 50;

const int initialScore = 25000;
const int robotPrice = 2000;// 普通机器人的价格
const int robotProPrice = 3000;// 高级机器人的价格
const int boatPrice = 8000;
const int boatProPrice = 20000;// 高级轮船的价格

const int maxRobotNum = 50;// 暂时只买一种机器人
const int maxBoatNum = 2;

const int valuableGoodThreshold = 750;// 普通货物价值 [1, 100], 贵重货物价值 [1400, 1600]

// 全局遍历邻接节点的方式: 右, 左, 上, 下
const vector<pair<int, int>> moves = {
        {0, 1}, {0, -1}, {-1, 0}, {1, 0}
};

const int maxBoat2DeliveryDistance = 480;// 构建深度图时，boat到delivery的最大距离
const int maxBoat2BerthDistance = 480;// 构建深度图时，boat到Berth的最大距离

const int minGoodWorth = 50;

const int maxGoodNum = maxFrameID * 11;

const int maxNumOfValuableGoodLocalDepth = 1500;
const int maxNumOfCheapGoodLocalDepth = 2500;
const int sideOfValuableGoodLocalDepth = 101;// 以货物为中心的栅格矩形的边长, 需要设置为奇数
const int sideOfCheapGoodLocalDepth = 101;

const int waitEpsilonFrame = 30;// 船等待的帧
const int waitWorth = 200;// 大于这个价值表示值得等待

// 使用下行宏定义代码声明变量
#define DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData) \
    auto frameID = globalData.frameID;\
    auto& money = globalData.money;\
    auto& berths = globalData.berths;\
    auto& robots = globalData.robots;\
    auto& boats = globalData.boats;\
    auto& goods = globalData.goods;\
    auto& goodsLeftPtr = globalData.goodsLeftPtr;\
    auto& privateRobots = globalData.privateRobots;\
    auto& privateBoats = globalData.privateBoats;\
    auto& askedRobots = globalData.askedRobots;

// 使用下列宏定义代码按序申请临界资源
#define ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData) \
    auto& sharedFrameID = llmData.sharedFrameID;\
    auto& sharedPrivateRobotAnswer = llmData.sharedPrivateRobotAnswer;\
    auto& sharedPrivateRobotQuestion = llmData.sharedPrivateRobotQuestion;\
    auto& sharedAskedPrivateRobots = llmData.sharedAskedPrivateRobots;\
    auto& sharedInformation = llmData.sharedInformation;\
    auto& sharedAskedCount = llmData.sharedAskedCount;\
    auto& sharedAnsCount = llmData.sharedAnsCount;

// 只处理 LLM 货物的时候, 使用下面宏定义; 注释可使得代码处理普通货物
#define ONLY_PROCESS_VALUABLE_GOODS

// 只购买高级机器人时, 使用下面宏定义; 注释可使得代码全买普通机器人
#define ONLY_BUY_ROBOT_PRO
