#pragma once
#include "ConstantDefinition.h"

struct Robot;
struct Goods;
struct Berth;
struct Boat;

struct Robot {
    /****************************      成员变量      ****************************/
    int id;// 机器人 ID
    int x, y;// 机器人坐标                                            由系统指定! 不可更改
    int goodStatus;// 机器人货物状态, 0 表示不携带货物, 1 表示携带货物      由系统指定! 不可更改
    int moveStatus;// 机器人行动状态, 0 表示恢复状态, 1 表示正常状态        由系统指定! 不可更改
    vector<string> orders;//命令
    vector<vector<int>> path;// 暂存路径
    vector<int> target;// 存储目标位置
    int goodID = -1;// 要取或取到的 货物id, -1 表示无目标

    // 机器人上一帧的状态(还未使用)
    int prevX = -1, prevY = -1;
    int prevGoodStatus = -1;
    int prevMoveStatus = -1;
    int prevGoodID = -1;

    /****************************      成员函数      ****************************/
    // 传入当前帧，对应的货物
    bool get();
    bool pull();
    bool move(int);
    bool reset();// 丢弃当前所有的信息, 只能对未携带货物的机器人起作用!
    void updatePrevData();

    /****************************      构造函数      ****************************/
    Robot() { }
    Robot(int id, int x, int y):id(id), x(x), y(y){

    }

};

struct Good {
    /****************************      成员变量      ****************************/
    int id;// 货物 ID
    int x, y;// 货物坐标
    int worth;// 货物价值
    int appearFrame;// 货物出现的帧
    int status = 0;// 货物状态, 0 表示初始状态, 1 表示被选中, 2 表示被携带
    int berthID;
    int dist2nearestBerth;
    //当距离足够近时使用货物自身的深度图
    vector<vector<int>> depthMap;

    /****************************      成员函数      ****************************/
    // 输入某帧
    bool isValid(int);

    /****************************      构造函数      ****************************/
    Good() {}
    Good(int id, int x, int y, int worth, int appearFrame):id(id), x(x), y(y), worth(worth), appearFrame(appearFrame){

    }
};

struct Berth {
    /****************************      成员变量      ****************************/
    int id;// 泊位 ID
    int x, y;// 泊位坐标
    int transportTime;// 运输速度, 表示该泊位轮船运输到虚拟点的时间(虚拟点轮船移动到泊位的时间同)
    int loadingSpeed;// 装载速度, 即每帧可以装载的物品数
    queue<int> goodsWorth;// 泊位上的货物价值列表
    int sumOfGoodsWorth = 0;// 当前泊位上的货物价值总和
    // berth 不需要指令

    /****************************      构造函数      ****************************/
    Berth() {}
    Berth(int id, int x, int y, int transportTime, int loadingSpeed):id(id), x(x), y(y), transportTime(transportTime), loadingSpeed(loadingSpeed){

    }
};

struct Boat {
    /****************************      成员变量      ****************************/
    int id;// 轮船 ID
    int capacity;// 船的容积, 表示最多能装的物品数
    int size = 0;// 船已使用的容积, m_size <= m_capacity
    int worth = 0;// 船上已有的货物价值
    int status;// 轮船状态, 0 表示移动中, 无需处理; 1 表示装货状态或者运输完成状态, 可以下达指令; 2 表示在泊位外等待, 无需处理
    int berthID;// 目标泊位 ID, -1 表示虚拟点
    string order;

    int orderFrame = -1;// 记录上一次收到指令的 frame id
    int prevBerthID = -1;// 记录上一次停留的泊位

    /****************************      成员函数      ****************************/
    bool ship(int berthID, int frameID = -1);
    bool go(int frameID = -1);

    void reset();

    /****************************      构造函数      ****************************/
    Boat() {}
    Boat(int id, int capacity):id(id), capacity(capacity){

    }

};

struct Target {
    int id;
    int worth;
    int takeCost;
    int sendCost;
};

bool targetCompare(const Target t1, const Target t2);