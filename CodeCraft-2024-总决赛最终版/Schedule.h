#include "RobotSchedule.h"
//#include "BoatSchedule_antalu.h"
#include "BoatSchedule.h"

class Schedule {
public:
    Schedule();

    void preProcess(MapFunction& mapFunction, GlobalData& globalData);

    void schedule(RobotSchedule& robotSchedule, BoatSchedule& boatSchedule, MapFunction& mapFunction, GlobalData& globalData);

    void buyRobots(MapFunction& mapFunction, GlobalData& globalData);

    void buyBoats(MapFunction& mapFunction, GlobalData& globalData);

    void postProcess(GlobalData& globalData);

    void summarize(GlobalData& globalData);

private:

    int m_actualFrameCount = 0;
    int m_scoreRobot2Berth = 0;// 所有机器人运输到泊位的货物价值
    int m_scoreBoat2Delivery = 0;// 所有轮船运输到交货点的货物价值

    int m_scoreOfCheapGood2Berth = 0;
    int m_scoreOfValuableGood2Berth = 0;

    int m_rcIdx = 0;
    int m_bcIdx = 2;
};