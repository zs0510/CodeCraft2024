#pragma once
#include "ConstantDefinition.h"

// 不可更改, 题目限制
enum struct MoveDirection: int {
    Right = 0,
    Left = 1,
    Up = 2,
    Down = 3
};

enum struct RotationDirection: int {
    Clockwise = 0,
    Anticlockwise = 1
};

enum struct OrderType: int {
    RobotGet = 0,
    RobotPull = 1,
    RobotMove = 2,
    RobotAns = 3,
    BoatDept = 4,
    BoatBerth = 5,
    BoatRot = 6,
    BoatShip = 7
};

struct Robot;
struct Good;
struct Berth;
struct Boat;
struct Order;
struct GlobalData;

struct Order {
    /****************************      成员变量      ****************************/
    OrderType orderType;
    int parameter;

    /****************************      构造函数      ****************************/
    Order() {}
    Order(OrderType _orderType, int _parameter = -1)
    : orderType(_orderType), parameter(_parameter) {

    }
};

struct Robot {
    /****************************      成员变量      ****************************/
    int id;// 机器人 ID
    int type = 1;// 机器人类型, 1 表示普通机器人(仅可携带 1 个货物), 2 表示高级机器人(可携带 2 个货物)
    int x, y;// 机器人坐标
    int goodsNum = 0;// 机器人携带的货物数量
    vector<Order> orders;// 命令
    int targetGoodID = -1; // 计划要去取的货物 ID
    stack<int> goodIDs; // 机器人身上已经携带的货物 ID

    int targetBerthID = -1;

    int prevGoodsNum = 0;

    set<int> neighborGoods;
    set<int> neighborRobots;

    // 记录前三帧的位置 防止机器人摆头
    int prevX1 = -1, prevY1 = -1;
    int prevX2 = -1, prevY2 = -1;
    int prevX3 = -1, prevY3 = -1;

    // 记录想去的位置
    int wantX1 = -1, wantY1 = -1;

    int realtimeGoodsNum = 0;

    /****************************      成员函数      ****************************/
    // 传入当前帧，对应的货物
    void get();
    void pull();
    void move(int);
    void reset();// 丢弃当前所有的信息, 只能对未携带货物的机器人起作用!
    void ans(int answer);

    /****************************      构造函数      ****************************/
    Robot() {}
    Robot(int id, int x, int y):id(id), x(x), y(y){

    }

};

struct Good {
    /****************************      成员变量      ****************************/
    int id;// 货物 ID
    bool valuable = false;
    int x, y;// 货物坐标
    int worth;// 货物价值
    int appearFrame;// 货物出现的帧

    int berthID = -1;
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
    int dir;
    int loadingSpeed;// 装载速度, 即每帧可以装载的物品数
    queue<int> goodsWorth;// 泊位上的货物价值列表
    int sumOfGoodsWorth = 0;// 当前泊位上的货物价值总和

    double weight;

    /****************************      构造函数      ****************************/
    Berth() {}
    Berth(int id, int x, int y, int loadingSpeed):id(id), x(x), y(y), loadingSpeed(loadingSpeed){

    }
};

struct Boat {
    /****************************      成员变量      ****************************/
    int id;// 轮船 ID
    int x, y;// 轮船坐标
    int dir;// 轮船方向
    int capacity;// 船的容积, 表示最多能装的物品数
    int goodsNum = 0;// 携带的货物数量, goodsNum <= capacity
    int worth = 0;// 船上已有的货物价值
    int status;// 轮船状态, 0 表示正常行驶状态; 1 表示恢复状态; 2 表示装载状态
    int berthID;// 目标泊位 ID
    int prevBerthID = -1;// 上一个去过的泊位ID
    vector<Order> order;// 为了方便写代码, 将此命令设置为一个向量, 注意: 轮船每帧只能执行一个命令

    int prevX = -1, prevY = -1;
    int prevDir = -1;

    // 防止某些傻逼的船装满了还不走
    int prevGoodsNum1 = 0, prevStatus1 = -1;
    int prevGoodsNum2 = 0, prevStatus2 = -1;
    int prevGoodsNum3 = 0, prevStatus3 = -1;
    int prevGoodsNum4 = 0, prevStatus4 = -1;
    int prevGoodsNum5 = 0, prevStatus5 = -1;
    int prevGoodsNum6 = 0, prevStatus6 = -1;
    int prevGoodsNum7 = 0, prevStatus7 = -1;
    int prevGoodsNum8 = 0, prevStatus8 = -1;
    int prevGoodsNum9 = 0, prevStatus9 = -1;
    int prevGoodsNum10 = 0, prevStatus10 = -1;

    /****************************      成员函数      ****************************/
    void dept();
    void berth();
    void rot(RotationDirection);
    void ship();

    /****************************      构造函数      ****************************/
    Boat() {}
    Boat(int id, int capacity):id(id), capacity(capacity){

    }

};

struct GlobalData {
    vector<Robot> robots;// 所有的机器人
    vector<Good> goods;// 所有的货物
    vector<Berth> berths;// 所有的泊位
    vector<Boat> boats;// 所有的轮船

    int frameID;
    char plat[szOfMap + 1][szOfMap + 1];
    int boatCapacity0 = 0;// 普通轮船容量, 同一局游戏中此值不变
    int boatCapacity1 = 0;// 高价值轮船容量, 同一局游戏中此值不变
    int numOfBerths = 0;// 泊位数量, 同一局游戏中此值不变
    int numOfRobots = 0;// 所有队伍机器人的个数
    int numOfBoats = 0;// 所有队伍轮船的个数

    int money;
    int goodsLeftPtr = 0;
    int numOfPrivateRobots = 0;// 本队伍机器人的个数
    int numOfPrivateBoats = 0;// 本队伍轮船的个数
    int numOfAskedRobots = 0;// 本队处于答题状态机器人的个数
    set<int> privateRobots;// 本队伍机器人的全局 ID
    set<int> privateBoats;// 本队伍轮船的全局 ID

    unordered_set<int> askedRobots;// 本队伍处于答题状态的机器人全局 ID

};