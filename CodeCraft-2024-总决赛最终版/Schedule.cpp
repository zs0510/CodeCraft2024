#include "Schedule.h"

Schedule::Schedule(){

}

void Schedule::schedule(RobotSchedule& robotSchedule, BoatSchedule& boatSchedule, MapFunction& mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    ++m_actualFrameCount;

    preProcess(mapFunction, globalData);

    robotSchedule.schedule(mapFunction, globalData);
    boatSchedule.schedule(mapFunction, globalData);

    postProcess(globalData);

    // TODO: 多少帧开始不买
    if(frameID <= 10000){
        buyBoats(mapFunction, globalData);
        buyRobots(mapFunction, globalData);
    }
}

void Schedule::buyRobots(MapFunction& mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

//    if (privateBoats.size() < 3) {
//        return;// 只在轮船数目大于等于 3 的时候才开始购买机器人
//    }

    auto m_robotCreatorBlock = mapFunction.getRobotCreatorBlocks();

    if(privateRobots.size() == 0){
        printf("lbot %d %d 1\n", m_robotCreatorBlock[3][0].first, m_robotCreatorBlock[3][0].second);

        printf("lbot %d %d 0\n", m_robotCreatorBlock[3][0].first, m_robotCreatorBlock[3][0].second);
        printf("lbot %d %d 0\n", m_robotCreatorBlock[4][0].first, m_robotCreatorBlock[4][0].second);
        printf("lbot %d %d 0\n", m_robotCreatorBlock[4][0].first, m_robotCreatorBlock[4][0].second);
        printf("lbot %d %d 0\n", m_robotCreatorBlock[5][0].first, m_robotCreatorBlock[5][0].second);
        printf("lbot %d %d 0\n", m_robotCreatorBlock[5][0].first, m_robotCreatorBlock[5][0].second);
        printf("lbot %d %d 0\n", m_robotCreatorBlock[6][0].first, m_robotCreatorBlock[6][0].second);
        printf("lbot %d %d 0\n", m_robotCreatorBlock[6][0].first, m_robotCreatorBlock[6][0].second);

    }
    else{
        int onceRobotCount = 8;
        int buyRobotCount = 0;

        unordered_set<int> usedRCIdx;

        int buyRobotPrice = robotPrice;

//#ifdef ONLY_BUY_ROBOT_PRO
//        buyRobotPrice = robotProPrice;
//#endif

        while (privateRobots.size() + buyRobotCount < maxRobotNum && money >= buyRobotPrice && onceRobotCount > 0) {
            if (usedRCIdx.size() == m_robotCreatorBlock.size()) {
                usedRCIdx.clear();// 保证能买机器人
            }
            m_rcIdx = rand() % m_robotCreatorBlock.size();
            while (usedRCIdx.count(m_rcIdx)) {
                m_rcIdx = rand() % m_robotCreatorBlock.size();
            }
            money -= buyRobotPrice;
            // 第一个元素是该机器人购买块的坐上角
//#ifndef ONLY_BUY_ROBOT_PRO
            printf("lbot %d %d 0\n", m_robotCreatorBlock[m_rcIdx][0].first, m_robotCreatorBlock[m_rcIdx][0].second);
//#else
//            printf("lbot %d %d 1\n", m_robotCreatorBlock[m_rcIdx][0].first, m_robotCreatorBlock[m_rcIdx][0].second);
//#endif
            cerr << frameID << endl;
            cerr << "买机器人" << endl;
            --onceRobotCount;
            ++buyRobotCount;
            usedRCIdx.insert(m_rcIdx);
        }
    }
    fflush(stdout);
}

void Schedule::buyBoats(MapFunction& mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    auto m_boatCreator = mapFunction.getBoatCreators();

    int onceBoatCount = 1;
    int buyBoatCount = 0;

    while (privateBoats.size() + buyBoatCount < maxBoatNum && money >= boatPrice && onceBoatCount > 0) {
        money -= boatPrice;
        printf("lboat %d %d 0\n", m_boatCreator[m_bcIdx].first, m_boatCreator[m_bcIdx].second);
        cerr << frameID << endl;
        cerr << "买船" << endl;
        // 每次都从不同的地方买boat
        m_bcIdx = (m_bcIdx + 1) % m_boatCreator.size();
        --onceBoatCount;
        ++buyBoatCount;
    }
    fflush(stdout);

}

void Schedule::preProcess(MapFunction& mapFunction, GlobalData& globalData) {
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    // 1. 根据机器人身上的货物数量变化情况更新信息
    for (auto& robot : robots) {
        robot.realtimeGoodsNum = robot.goodsNum;
        // TODO: 考虑机器人同一帧先取货物再卸货物?
        if (robot.goodsNum > robot.prevGoodsNum) {
            // 机器人上一帧取了货物
            // 有两种情况 1.1 先取货物再移动; 1.2 先移动再取货物
            // 第 3 种情况: 先取货物再移动再取货物
            int gid =-1;
            if (mapFunction.isDisappearedGoods(robot.prevX1, robot.prevY1)) {
                gid = mapFunction.getGoodID(robot.prevX1, robot.prevY1);
            } else if (mapFunction.isDisappearedGoods(robot.x, robot.y)) {
                gid = mapFunction.getGoodID(robot.x, robot.y);
            }
            if (gid != -1) {
                robot.goodIDs.push(gid);
            } else {
                cerr << "gid = -1\n";
            }
        } else if (robot.goodsNum < robot.prevGoodsNum) {
            // 机器人上一帧卸了货物
            // 有两种情况 1.1 先卸货物再移动; 1.2 先移动再卸货物
            if (robot.goodIDs.empty()) {
                cerr << "robot.goodIDs.empty()\n";
                continue;
            }

            int berthID = mapFunction.getBerthID(robot.x, robot.y);
            if (berthID == -1) {
                berthID = mapFunction.getBerthID(robot.prevX1, robot.prevY1);
            }
            if (berthID != -1) {
                while (!robot.goodIDs.empty()) {
                    int gid = robot.goodIDs.top();
                    robot.goodIDs.pop();
                    berths[berthID].goodsWorth.push(goods[gid].worth);
                    berths[berthID].sumOfGoodsWorth += goods[gid].worth;
                    m_scoreRobot2Berth += goods[gid].worth;
                    if (goods[gid].valuable) {
                        m_scoreOfValuableGood2Berth += goods[gid].worth;
                    } else {
                        m_scoreOfCheapGood2Berth += goods[gid].worth;
                    }
                }
            } else {
                cerr << "robot->berthID = -1\n";
            }
        }
    }

    // 2. 根据轮船上的货物数量变化情况更新信息
    for (auto& boat : boats) {
        if (boat.goodsNum > boat.prevGoodsNum1) {
            // 轮船上一帧装了货物
            int loadingNum = boat.goodsNum - boat.prevGoodsNum1;
            int berthID = mapFunction.getBerthID(boat.x, boat.y);
            if (berthID == -1) {
                berthID = mapFunction.getBerthID(boat.prevX, boat.prevY);
            }
            if (berthID != -1) {
                for (int i = 0; i < loadingNum; ++i) {
                    auto worth = berths[berthID].goodsWorth.front();
                    berths[berthID].goodsWorth.pop();
                    berths[berthID].sumOfGoodsWorth -= worth;
                    boat.worth += worth;
                }
            } else {
                cerr << "boat->berthID = -1\n";
            }
        } else if (boat.goodsNum < boat.prevGoodsNum1) {
            // 轮船上一帧卸了货物
            // TODO: 统计分数
            m_scoreBoat2Delivery += boat.worth;
            boat.worth = 0;
        }
    }

    // 3. 更新地图: 删除消失的货物以及为新货物创建局部深度图
    mapFunction.update(globalData);

}

void Schedule::postProcess(GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    for(auto& robot : robots){
        robot.prevX3 = robot.prevX2;
        robot.prevY3 = robot.prevY2;

        robot.prevX2 = robot.prevX1;
        robot.prevY2 = robot.prevY1;

        robot.prevX1 = robot.x;
        robot.prevY1 = robot.y;

        robot.prevGoodsNum = robot.goodsNum;
    }

    for(auto& boat : boats){
        boat.prevX = boat.x;
        boat.prevY = boat.y;
        boat.prevDir = boat.dir;

        boat.prevGoodsNum10 = boat.prevGoodsNum9;
        boat.prevGoodsNum9 = boat.prevGoodsNum8;
        boat.prevGoodsNum8 = boat.prevGoodsNum7;
        boat.prevGoodsNum7 = boat.prevGoodsNum6;
        boat.prevGoodsNum6 = boat.prevGoodsNum5;
        boat.prevGoodsNum5 = boat.prevGoodsNum4;
        boat.prevGoodsNum4 = boat.prevGoodsNum3;
        boat.prevGoodsNum3 = boat.prevGoodsNum2;
        boat.prevGoodsNum2 = boat.prevGoodsNum1;
        boat.prevGoodsNum1 = boat.goodsNum;

        boat.prevStatus10 = boat.prevStatus9;
        boat.prevStatus9 = boat.prevStatus8;
        boat.prevStatus8 = boat.prevStatus7;
        boat.prevStatus7 = boat.prevStatus6;
        boat.prevStatus6 = boat.prevStatus5;
        boat.prevStatus5 = boat.prevStatus4;
        boat.prevStatus4 = boat.prevStatus3;
        boat.prevStatus3 = boat.prevStatus2;
        boat.prevStatus2 = boat.prevStatus1;
        boat.prevStatus1 = boat.status;
    }

}

void Schedule::summarize(GlobalData &globalData) {
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    // 打印泊位信息
    // cerr << "\n*********************\t Information of Berths \t****************************\n";
    int sumBerthsWorth = 0;
    for (auto& berth : berths) {
//        cerr << "berth[" << berth.id << "]: remain worth = " << setw(6) << berth.sumOfGoodsWorth
//             << ", \tremain num = " << setw(3) << berth.goodsWorth.size() << endl;
        sumBerthsWorth += berth.sumOfGoodsWorth;
    }

    int sumBoatsWorth = 0;
    for (auto& boat : boats) {
        sumBoatsWorth += boat.worth;
    }

    cerr << "\n*********************\t Information of Goods \t****************************\n";
    cerr << "\tsum of cheap good to berth worth: " << m_scoreOfCheapGood2Berth << endl;
    cerr << "\tsum of valuable good to berth worth: " << m_scoreOfValuableGood2Berth << endl;

    cerr << "\n*********************\t Information of Berths \t****************************\n";
    cerr << "\tsum of berths worth: " << sumBerthsWorth << endl;

    cerr << "\n*********************\t Information of Boats \t****************************\n";
    cerr << "\tsum of boats worth: " << sumBoatsWorth << endl;

    cerr << "\n*********************\t Information of Scores \t****************************\n";
    cerr << "\tscore of robot to berth: " << m_scoreRobot2Berth << endl;
    cerr << "\tscore of boat to delivery: " << m_scoreBoat2Delivery << endl;

    int costOfBuyRobots = 0;
    int robotCount = 0, robotProCount = 0;
    for (auto& rid : privateRobots) {
        if (robots[rid].type == 2) {
            costOfBuyRobots += robotProPrice;
            ++robotProCount;
        } else {
            costOfBuyRobots += robotPrice;
            ++robotCount;
        }
    }
    int costOfBuyBoats = 0;
    int boatCount = 0, boatProCount = 0;
    for (auto& bid : privateBoats) {
        if (boats[bid].capacity > 30) {
            costOfBuyBoats += boatProPrice;
            ++boatProCount;
        } else {
            costOfBuyBoats += boatPrice;
            ++boatCount;
        }
    }
    cerr << "\n*********************\t Robot0 count = " << setw(6) << robotCount << "\t****************************\n";
    cerr << "\n*********************\t Robot1 count = " << setw(6) << robotProCount << "\t****************************\n";
    cerr << "\n*********************\t Boat0 count = " << setw(6) << boatCount << "\t****************************\n";
    cerr << "\n*********************\t Boat1 count = " << setw(6) << boatProCount << "\t****************************\n";
    cerr << "\n*********************\t Frame count = " << setw(6) << m_actualFrameCount << "\t****************************\n";
    cerr << "\n*********************\t Judge score = "
         << setw(6) << m_scoreBoat2Delivery + initialScore - costOfBuyRobots - privateBoats.size() * boatPrice
         << "\t****************************\n";

}