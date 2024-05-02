#pragma once
#include <thread>
#include "DataStruct.h"

class MapFunction {
public:
    explicit MapFunction(GlobalData& globalData);

    void update(GlobalData& globalData);// 更新每帧的信息

    void removeGood(int x, int y); // 暂存过期 或 被拿走的货物

    bool isDisappearedGoods(int x, int y);
    int getGoodID(int x, int y);

    int getGoodRobotDepth(int goodID, int x, int y);// 取good到机器人的局部深度
    int getBerthRobotDepth(int id, int x, int y);// 取机器人到berth的深度
    int getBerthBoatDepth(int id, int x, int y, int dir);// 取船到berth的深度
    int getDeliveryDepth(int x, int y, int dir, int deliveryID = -1);// -1 返回全局的最小深度, 否则返回该 ID 对应的深度

    int getNearBerthID(int x, int y);
    int getBerthID(int x, int y);

    int getMinDistBerth2Delivery(int id);

    const vector<vector<pair<int, int>>>& getRobotCreatorBlocks();
    const vector<pair<int, int>>& getBoatCreators();

    bool canMoveOn(int x, int y);// 不是障碍 & 合法 即返回 true
    bool robotCanMoveOn(int x, int y);// 检查机器人可否移到该位置
    bool isSea(int x, int y);// 返回该点是否是轮船的身体可达的
    bool isMainRoad(int x, int y);// 检查是否为主干道
    bool isMainChannel(int x, int y);// 检查是否为主航道
    bool isOnMainChannel(int x, int y, int dir);// 检查轮船是否位于主航道
    bool checkBoatCanMoveOn(int x, int y, int dir);// 检查轮船能否移动到该位置

    // 轮船函数
    int getBoatDir(int currentDir, RotationDirection rotation);
    vector<pair<int, int>> getBoatBody(int x, int y, int dir);
    vector<int> getShip(int x, int y, int dir);
    vector<int> getClockwise(int x, int y, int dir);
    vector<int> getAnticlockwise(int x, int y, int dir);
    vector<int> getInvShip(int x, int y, int dir);
    vector<int> getInvClockwise(int x, int y, int dir);
    vector<int> getInvAnticlockwise(int x, int y, int dir);

    void addAnsweredGood(int goodID);
    bool isAnsweredGood(int goodID);

    map<pair<int, int>, int> m_pos2goodID;// 货物坐标 到 货物ID 的映射

private:
    int goodLeftIndex = 0;

    void initDelivery(GlobalData& globalData);
    void initDeliveryDepth(GlobalData& globalData, int left, int right);
    void initBerth(GlobalData& globalData);
    void initBerthDepth(GlobalData& globalData, int left, int right);
    void initBerthWeight(GlobalData& globalData);

    void initRobotCreator(GlobalData& globalData);
    void initBoatCreator();

    void initGoodLocalDepth();

    void updateGoodLocalDepth(GlobalData& globalData);

    void calcGoodLocalDepth(const vector<pair<int, int>>& buildGoodDepth, int left, int right);
    void calcSingleGoodLocalDepth(int goodID, int depthID);

    bool isValidLocalCoord(int goodID, int localX, int localY);
    pair<int, int> getWorldCoord(int goodID, int localX, int localY);
    pair<int, int> getLocalCoord(int goodID, int worldX, int worldY);

    void removeGoodLocalDepth(int goodID);

    vector<vector<char>> m_map;//地图

    vector<vector<pair<int, int>>> m_robotCreatorBlock; // 机器人出生点块
    vector<pair<int, int>> m_boatCreator; // 船出生点块

    vector<vector<pair<int, int>>> m_deliveryBlock; // 交货点块

    vector<vector<vector<int>>> m_berthRobotDepth;// 泊位的陆地深度图
    vector<vector<vector<vector<int>>>> m_berthBoatDepth;// 泊位的海洋深度图
    vector<vector<vector<vector<int>>>> m_deliveryBlockGlobalDepth; // 交货点块的轮船深度图

    // 货物的局部深度图数据结构 ***** begin *****
    int m_valuableGoodLocalDepth[maxNumOfValuableGoodLocalDepth][sideOfValuableGoodLocalDepth][sideOfValuableGoodLocalDepth];
    int m_cheapGoodLocalDepth[maxNumOfCheapGoodLocalDepth][sideOfCheapGoodLocalDepth][sideOfCheapGoodLocalDepth];
    bool m_goodIsValid[maxGoodNum];// 记录货物是否已经无效, true 表示有效, false 表示无效(以避免构造此前在队列中, 但后面被取走的货物的深度图)
    bool m_goodIsValuable[maxGoodNum];// 记录货物是否是高价值货物, true 表示是高价值货物, false 表示非高价值货物
    unordered_map<int, int> m_valuableGoodID2LocalDepthID;
    unordered_map<int, int> m_cheapGoodID2LocalDepthID;
    queue<int> m_emptyValuableGoodLocalDepthID;
    queue<int> m_emptyCheapGoodLocalDepthID;
    queue<int> m_waitingValuableGoodBuildLocalDepth;
    queue<int> m_waitingCheapGoodBuildLocalDepth;
    // 货物的局部深度图数据结构 ***** end *****

    vector<pair<int, int>> m_goodLocation;// 货物的位置

    set<pair<int, int>> disappearedGoods;// 上一帧消失的货物

    map<pair<int, int>, int> m_pos2deliveryBlockID; // 坐标 到 交货点块ID 的映射

    map<pair<int, int>, int> m_pos2Berth;// 泊位坐标, 坐标到泊位 ID 的映射
    map<pair<int, int>, int> m_pos2NearBerth;// 靠泊区坐标, 靠泊区坐标到泊位 ID 的映射

    vector<vector<int>> m_distBerth2Delivery;// berth到交货点的距离
    vector<int> m_minDistBerth2Delivery;// berth到交货点的最短距离

    unordered_set<int> m_answeredGoods;// 已尝试回答过的货物

};