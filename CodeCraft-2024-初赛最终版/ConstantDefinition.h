#pragma once

#if defined(_WIN32) || defined(__APPLE__)
// 本地运行(Windows / MacOS)
#include <iostream>
#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#else
// 服务器运行(Linux)
#include <bits/stdc++.h>
#endif
#include <sys/time.h>

using namespace std;

const int INF = 0x3fffffff;// 无穷大的整数

const int szOfMap = 200;
const int numOfRobots = 10;
const int numOfBerths = 10;
const int numOfBoats = 5;

// 货物存在的帧率
const int frameOfGoodExist = 1000;

// 最大的帧 ID
const int maxFrameID = 15000;
const int framesFromBerth2Berth = 500;

//////////////////////////////////////
//              可调参数
//////////////////////////////////////

//防止跳帧导致货物无法准时送达
//const int epsilonFrame = 0;

//机器人防跳帧和避让
const int robotEpsilonFrame1 = 6;
//机器人防跳帧和避让
const int robotEpsilonFrame2 = 6;
//机器人防跳帧和避让
const int robotEpsilonFrame3 = 3;
//船防跳帧
const int boatEpsilonFrame = 1;

//深情系数 用来防止机器人摆头 大于1即可，不要太大
const double deepLoveCoefficient1 = 1.3;
const double deepLoveCoefficient2 = 1.3;
const double deepLoveCoefficient3 = 1.34;

//true表示使用匈牙利算法 false表示使用贪心策略
const bool hungarian = true;

// 全局遍历邻接节点的方式: 右, 左, 上, 下
//const vector<pair<int, int>> moves = {
//        {0, 1}, {0, -1}, {-1, 0}, {1, 0}
//};

// {0, 1}对应的索引为0
// {0, -1}对应的索引为1
// {-1, 0}对应的索引为2
// {1, 0}对应的索引为3
// 赛题中给出的方向按照上面的顺序，因此需要directionMap对其进行转译
// 需要注意的是 我使用了方向锁 所以 0和1 2和3是绑定的
const vector<pair<int, int>> moves = {
        {0, 1}, {0, -1}, {-1, 0}, {1, 0}
};

// 根据对应的索引填充directionMap即可
const vector<int> directionMap = {0, 1, 2, 3};

//下面的权重越小表示优先级更高
//几个排列组合
// 0    10    20
// 10    0    20
// 0     1     2
// 1     0     2

//送货机器人的权重
const int sendingRobotsPriority = 0;

//带机器人的权重
const int takingRobotsPriority = 10;

//没事干机器人的权重
const int freeRobotsPriority = 20;

const double boatCoff = 0.2;