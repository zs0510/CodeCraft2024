#include "MapFunction.h"
#include "HungarianAlgorithm.h"

#include <random>

class BoatSchedule {
public:
    BoatSchedule();

    // 船舶调度
    void schedule(MapFunction& mapFunction, GlobalData& globalData);

private:
    void preProcess(MapFunction& mapFunction, GlobalData& globalData);
    // 为船舶设定目标
    void optBoats(MapFunction& mapFunction, GlobalData& globalData);

    // 移动船舶
    void moveBoats(MapFunction& mapFunction, GlobalData& globalData);

    // 每条船只检查是否会与id比自身大的船发生碰撞
    map<pair<int, int>, set<int>> m_boatLock;

    // 去berth的船id
    set<int> boatsGoBerth;
    // 去delivery的船id
    set<int> boatsGoDelivery;

    int boatSendWorth;
};
