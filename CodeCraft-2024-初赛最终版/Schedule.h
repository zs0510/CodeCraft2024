#pragma once
#include <unordered_map>
#include <set>
#include <map>
#include "DataStruct.h"
#include "AStar.h"
#include "HungarianAlgorithm.h"

// Create by Zhang

class Schedule {
public:
    explicit Schedule(vector<vector<char>>& plat, const vector<Berth>& berths, const vector<Robot>& robots): m_map(plat){
        initPartition(berths);
        initBerthDepth(berths);

        m_squareLength = 4;

        initSquare();
        initSquareConnect();
        initSquareDepth();

        identifyMapID();
    }

    void optimize(int frameID,
                  vector<Robot>& robots,
                  vector<Good>& goods,
                  vector<Berth>& berths,
                  vector<Boat>& boats,
                  AStar* aStar,
                  int& goodsLeftPtr);

    void summarize(int frameID,
                   vector<Robot>& robots,
                   vector<Good>& goods,
                   vector<Berth>& berths,
                   vector<Boat>& boats,
                   AStar* aStar,
                   int& goodsLeftPtr);

private:
    /****************************      调度函数      ****************************/

    void optRobotsWithGood(int frameID,
                           vector<Robot>& robots,
                           vector<Good>& goods,
                           vector<Berth>& berths,
                           vector<Boat>& boats,
                           AStar* aStar,
                           int& goodsLeftPtr);

    void optBoats(int frameID,
                  vector<Robot>& robots,
                  vector<Good>& goods,
                  vector<Berth>& berths,
                  vector<Boat>& boats,
                  AStar* aStar,
                  int& goodsLeftPtr);

    void optRobotsWithoutGoodDebug(int frameID,
                                   vector<Robot>& robots,
                                   vector<Good>& goods,
                                   vector<Berth>& berths,
                                   vector<Boat>& boats,
                                   AStar* aStar,
                                   int& goodsLeftPtr);

    /****************************      寻路函数      ****************************/
    void moveRobots(int frameID,
                    vector<Robot>& robots,
                    vector<Good>& goods,
                    vector<Berth>& berths);

    /****************************      成员函数      ****************************/

    // 根据上一帧信息与这一帧信息对整个系统进行仿真
    void simulate(int frameID,
                  vector<Robot>& robots,
                  vector<Good>& goods,
                  vector<Berth>& berths,
                  vector<Boat>& boats);

    void updateLockMap(int frameID,
                       vector<Robot>& robots,
                       vector<Good>& goods,
                       vector<Berth>& berths,
                       vector<Boat>& boats,
                       AStar* aStar,
                       int& goodsLeftPtr);

    bool canMoveOn(int x, int y);

    void initPartition(const vector<Berth>& berths);

    void initBerthDepth(const vector<Berth>& berths);

    void initIgnoreBerths(int frameID, const vector<Robot>& robots, const vector<Berth>& berths, const vector<Boat>& boats);

    void initDepthMap(int srcX, int srcY, int trgX, int trgY, vector<vector<int>>& depth);

    bool existSufficientBoat(Berth berth, vector<Boat>& boats, int ignoreBoatID = -1);// 判断一个泊位是否已有轮船为其提供足够的服务

    void initSquare();
    void initSquareConnect();
    void initSquareDepth();

    void identifyMapID();

    /****************************      成员变量      ****************************/
    vector<vector<char>> m_map;// 全局地图

    vector<vector<int>> m_partition;// 每个栅格对应的分区, -1 表示不与泊位连通

    vector<vector<vector<int>>> m_berthDepth;// 每个泊位的深度图

    vector<vector<int>> m_nearestBerthDist;// 每个栅格到泊位的最近距离

    map<pair<int, int>, int> m_pos2Berth;// 坐标到泊位 ID 的映射

    unordered_map<int, vector<vector<int>>> m_goodsDepth;// 商品的深度图(暂弃)

    // 记录优化器实际处理的帧数
    int m_actualFrameCount = 0;

    // 在 frameIgnoreBerth 后, 只考虑不在 ignoreBerths 中的 berth
    const int m_frameIgnoreBerth = 13000;
    const int m_ignoreNumOfBerth = 5;
    vector<int> m_scoreOfBerth = vector<int>(numOfBerths, 0);
    unordered_set<int> m_ignoreBerths;

    int m_scoreOfRobot2Berth = 0;// 机器人运输到泊位的价值
    int m_scoreOfBoat2Virtual = 0;// 轮船运输到虚拟点的价值

    int m_squareLength;
    //按5*5划分1600个区域,每个区域取中心计算深度图(如果不行就随机取点)
    //区域编号 分区编号(可能一个区域内有两个分区,需要分别做一次深度图)
    vector<vector<vector<vector<int>>>> m_squareDepth;
    vector<vector<pair<int, int>>> m_squareConnectCenter;
    vector<vector<int>> m_pos2Square;// 坐标到区域的映射
    vector<vector<int>> m_pos2SquareConnect;// 坐标到区域联通分量的映射

    map<pair<int, int>, int> m_directionLockMap; //方向锁
    set<pair<int, int>> m_lockMap; //普通锁

    int m_mapID;
    int robotEpsilonFrame;
};