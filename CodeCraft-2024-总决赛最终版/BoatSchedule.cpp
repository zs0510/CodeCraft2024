//
// Created by 许晨浩 on 2024/4/16.
//

#include "BoatSchedule.h"

BoatSchedule::BoatSchedule(){

}

void BoatSchedule::schedule(MapFunction& mapFunction, GlobalData& globalData){
    preProcess(mapFunction, globalData);

    // 船舶调度
    optBoats(mapFunction, globalData);
    // 船舶寻路
    moveBoats(mapFunction, globalData);

}

void BoatSchedule::preProcess(MapFunction& mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    //清除上一帧的锁
    m_boatLock.clear();

    //设置这一帧的锁
    for(auto& boat : boats){
        // 锁定船的当前位置
        auto boatBody = mapFunction.getBoatBody(boat.x, boat.y, boat.dir);
        for(auto& [x, y] : boatBody){
            m_boatLock[{x, y}].insert(boat.id);
        }

        for (int i = 0; i < 3; ++i) {
            vector<int> nextBoat;
            if (i == 0) {
                nextBoat = mapFunction.getShip(boat.x, boat.y, boat.dir);
            } else if (i == 1) {
                nextBoat = mapFunction.getClockwise(boat.x, boat.y, boat.dir);
            } else if (i == 2) {
                nextBoat = mapFunction.getAnticlockwise(boat.x, boat.y, boat.dir);
            }

            auto nextBoatBody = mapFunction.getBoatBody(nextBoat[0], nextBoat[1], nextBoat[2]);

            // 锁定船的所有可能移动的位置
            for(auto& [x, y] : nextBoatBody){
                m_boatLock[{x, y}].insert(boat.id);
            }
        }
    }

    // 快船冲出重围 不考虑碰撞
    if(frameID <= 100)
        m_boatLock.clear();

    //清除上一帧的调度
    boatsGoBerth.clear();
    boatsGoDelivery.clear();
}

void BoatSchedule::optBoats(MapFunction& mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    vector<int> antaluList;// 初始情况下 所有货物没装满的船都位于这个list中

    // 找出所有被占用的berth
    set<int> occupiedBerth;

//    // 当船过多时
//    if(boats.size() >= 60){
//        for (auto& boat : boats) {
//            // berth已被占用 不参与匹配
//            if(mapFunction.getBerthID(boat.x, boat.y) >= 0 && boat.status == 2 && boat.goodsNum <= 0.8*boat.capacity){
//                occupiedBerth.insert(mapFunction.getBerthID(boat.x, boat.y));
//            }
//        }
//    }
//    else{
    for (auto& boat : boats) {
        // berth已被占用 且 上面的船处于装货状态 不参与匹配
        if(mapFunction.getBerthID(boat.x, boat.y) >= 0 && boat.status == 2){
            occupiedBerth.insert(mapFunction.getBerthID(boat.x, boat.y));
        }
    }
//    }


    // 找出所有死掉的berth
    set<int> deadBerth;
    for(auto& boat : boats){
        // 故意卡着优势泊位恶心人
        // 10帧前货已经装满 且 现在还处于装货状态
        if(boat.prevStatus10 == 2 && boat.prevGoodsNum10 == boat.capacity && boat.status == 2){
            deadBerth.insert(mapFunction.getBerthID(boat.x, boat.y));
        }
    }

    // 找出所有即将有较高货物送来的berth
    // TODO: 目前还没有用到
    vector<int> berthComingWorth(berths.size(), 0);
    set<int> goodsComingBerth;
    for(auto& robot : robots){
        // 机器人未携带货物
        if(robot.goodsNum == 0)
            continue;

        int takeWorth = 0;
        auto goodIDs = robot.goodIDs;
        while(!goodIDs.empty()){
            auto top = goodIDs.top();
            goodIDs.pop();

            takeWorth += goods[top].worth;
        }

        int minDist = INF;
        int berthID = -1;
        for(auto& berth : berths){
            auto dist = mapFunction.getBerthRobotDepth(berth.id, robot.x, robot.y);
            if(dist < minDist){
                minDist = dist;
                berthID = berth.id;
            }
        }

        // berth有效 小于等待阈值
        if(berthID >= 0 && minDist <= waitEpsilonFrame){
            int prevDist1 = mapFunction.getBerthRobotDepth(berthID, robot.prevX1, robot.prevY1);
            int prevDist2 = mapFunction.getBerthRobotDepth(berthID, robot.prevX2, robot.prevY2);

            // 与berth的距离在逐渐减小
            if(minDist <= prevDist1 && prevDist1 <= prevDist2 && abs(minDist - prevDist1) + abs(prevDist1 - prevDist2) > 0){
                berthComingWorth[berthID] += takeWorth;
            }
        }
    }

    for(auto& berth : berths){
        if(berthComingWorth[berth.id] >= waitWorth){
            goodsComingBerth.insert(berth.id);
        }
    }

    // 找出需要去交货点的轮船
    for (auto boatID : privateBoats) {
        auto& boat = boats[boatID];

        int dist2Delivery = mapFunction.getDeliveryDepth(boat.x, boat.y, boat.dir);

        // 时间不多，船必须立刻去交货点
        if (boat.goodsNum > 0){
            // 处于移动状态
            if(boat.status == 0 && frameID + dist2Delivery + epsilonFrame >= maxFrameID){
                // 需要引路
                boatsGoDelivery.insert(boatID);
                continue;
            }
                // 处于装货状态，需要更多的epsilonFrame
            else if(boat.status == 2 && frameID + dist2Delivery + epsilonFrame*2 >= maxFrameID){
                // 不需要引路
                boat.dept();
                boat.berthID = -1;
                continue;
            }
        }

        // 检查轮船状态
        switch (boat.status) {
            case 0:
            {
                // 正常行驶状态
                // 1. 去交货点
                if (boat.goodsNum >= boat.capacity || (boat.goodsNum >= 0.9 * boat.capacity && mapFunction.getDeliveryDepth(boat.x, boat.y, boat.dir) <= 150)) {
                    boatsGoDelivery.insert(boatID);
                    continue;
                }
                // TODO: 机器人或船数量不足 钱刚好能买机器人或船

                // 2. 检查是否到达泊位
                int berthID = -1;
                if (mapFunction.getNearBerthID(boat.x, boat.y) >= 0) {
                    berthID = mapFunction.getNearBerthID(boat.x, boat.y) ;
                }
                if (berthID == -1 && mapFunction.getBerthID(boat.x, boat.y) >= 0) {
                    berthID = mapFunction.getBerthID(boat.x, boat.y);
                }
                // 是要去的berth
                if (boat.berthID == berthID && berthID != -1 && !occupiedBerth.count(berthID)) {
                    boat.berth();
                    // 占用该berth
                    occupiedBerth.insert(berthID);
                } else {
                    // 未到泊位, 仍需要匹配
                    antaluList.push_back(boatID);
                }

            }
                break;

            case 1:
            {
                // 船在靠泊或者离泊
                if(mapFunction.getBerthID(boat.x, boat.y) >= 0){

                }
                else{
                    // 未到泊位, 仍需要匹配
                    antaluList.push_back(boatID);
                }

            }
                break;

            case 2:
            {
                // 装载状态(指在泊位?)
                int berthID = mapFunction.getBerthID(boat.x, boat.y);
                boat.prevBerthID = berthID;

                // 目前的策略是装完走
//                if(boat.goodsNum == boat.capacity){/
                if((boat.prevStatus1 == 2 && boat.prevGoodsNum1 == boat.goodsNum && !goodsComingBerth.count(berthID)) || boat.goodsNum == boat.capacity){
                    boat.dept();
                    boat.berthID = -1;
                    continue;
                }

            }
                break;
        }

    }

    // 快船冲出重围
    if(frameID <= 100){

        // 只去优势泊位
        for (auto& boatID : antaluList) {

            auto& boat = boats[boatID];

            int distBoat2Berth20 = mapFunction.getBerthBoatDepth(20, boat.x, boat.y, boat.dir);
            int distBoat2Berth21 = mapFunction.getBerthBoatDepth(21, boat.x, boat.y, boat.dir);
            int distBoat2Berth26 = mapFunction.getBerthBoatDepth(26, boat.x, boat.y, boat.dir);
            int distBoat2Berth27 = mapFunction.getBerthBoatDepth(27, boat.x, boat.y, boat.dir);

            int minDist = min(min(distBoat2Berth20, distBoat2Berth21), min(distBoat2Berth26, distBoat2Berth27));

            // 判断最近的优势泊位有没有死掉
            if(distBoat2Berth20 == minDist && !deadBerth.count(20)){
                boat.berthID = 20;
                boatsGoBerth.insert(boatID);
            }
            else if(distBoat2Berth21 == minDist && !deadBerth.count(21)){
                boat.berthID = 21;
                boatsGoBerth.insert(boatID);
            }
            else if(distBoat2Berth26 == minDist && !deadBerth.count(26)){
                boat.berthID = 26;
                boatsGoBerth.insert(boatID);
            }
            else if(distBoat2Berth27 == minDist && !deadBerth.count(27)){
                boat.berthID = 27;
                boatsGoBerth.insert(boatID);
            }

        }
    }
    else{
        vector<int> matchBoatList = antaluList;

        // 优化剩余轮船与泊位之间的匹配
        vector<tuple<double, int, int>> weightBoatBerth;
        vector<tuple<double, int, int>> distBoatBerth;

        for (auto& boatID : matchBoatList) {
            auto& boat = boats[boatID];
            for (auto& berth : berths) {
                // 只匹配空的泊位
                if(occupiedBerth.count(berth.id))
                    continue;

                // 1. 计算轮船 去泊位 + 去交货点 的总距离
                int distBoat2Berth = mapFunction.getBerthBoatDepth(berth.id, boat.x, boat.y, boat.dir);
                int distBoat2Delivery = mapFunction.getMinDistBerth2Delivery(berth.id);
                int remainFrame = maxFrameID - frameID - distBoat2Berth - distBoat2Delivery;
                // 来不及运货到交货点
                if (remainFrame <= epsilonFrame)
                    continue;

                //把预估会到达的货物页插入其中
                auto goodsWorthCopy = berth.goodsWorth;

                // TODO: 计算泊位与对应的陆地面积哪个更大
                double goodsWorth = berth.weight;

                int berthGoodsCount = berth.goodsWorth.size();
                int maxAccommodateCount = boat.capacity - boat.goodsNum;// 轮船最多能装载的货物数量
                int maxLoadingCount = remainFrame * berth.loadingSpeed;// 为保证到达交货点, 最多能装载的货物数量
                // 真正能带走的货物数量
                int goodsCount = min(berthGoodsCount, min(maxAccommodateCount, maxLoadingCount));

                // 精确估计真正能带走的货物价值
                while(goodsCount--){
                    goodsWorth += goodsWorthCopy.front();
                    goodsWorthCopy.pop();
                }

                // 真正需要花费的时间
                //TODO: 这里的distBoat2Delivery的权重可以直接去掉
                int goodsCost = distBoat2Berth + distBoat2Delivery / INF  + (goodsCount + berth.loadingSpeed - 1) / berth.loadingSpeed;
                double weight = (double)goodsWorth / max(goodsCost, 1);
                if (boat.berthID == berth.id) {
                    weight *= 1.5;
                }
                if (boat.prevBerthID == berth.id) {
                    weight *= 0;
                }
                weightBoatBerth.emplace_back(weight, boatID, berth.id);
                distBoatBerth.emplace_back((double)1.0 / max(distBoat2Berth, 1), boatID, berth.id);
            }
        }

        // 计算轮船与泊位之间的匹配
        HungarianAlgorithm algBoatBerth(weightBoatBerth);
//        HungarianAlgorithm algBoatBerth(distBoatBerth);
        auto matchedBoatBerth = algBoatBerth.getMatchedRobotAndTarget();

        for (auto [boatID, berthID] : matchedBoatBerth) {
            auto& boat = boats[boatID];
            boat.berthID = berthID;
            boatsGoBerth.insert(boat.id);
        }
    }

}

void BoatSchedule::moveBoats(MapFunction& mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    for(auto& boatID : privateBoats){
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // 去berth船舶寻路
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        auto& boat = boats[boatID];
        if(boat.status == 1)
            continue;

        // 随机生成某个范围的数字
        std::vector<int> numbers(3);
        std::iota(numbers.begin(), numbers.end(), 0);
        std::random_device rd;
        std::mt19937 g(rd());
        // 前150帧按照直行优先走
        if(frameID > 150){
            std::shuffle(numbers.begin(), numbers.end(), g);
        }

        if(boatsGoBerth.count(boatID)){

            int flag = -1, minDepth = INF;
            for (int i = 0; i < 3; ++i) {
                vector<int> nextBoat;
                if (numbers[i] == 0) {
                    nextBoat = mapFunction.getShip(boat.x, boat.y, boat.dir);
                } else if (numbers[i] == 1) {
                    nextBoat = mapFunction.getClockwise(boat.x, boat.y, boat.dir);
                } else if (numbers[i] == 2) {
                    nextBoat = mapFunction.getAnticlockwise(boat.x, boat.y, boat.dir);
                }

                // 轮船能移动到该位置
                if (!mapFunction.checkBoatCanMoveOn(nextBoat[0], nextBoat[1], nextBoat[2])) {
                    continue;
                }

                // 轮船不会与其他轮船发生碰撞
                auto boatBody = mapFunction.getBoatBody(nextBoat[0], nextBoat[1], nextBoat[2]);
                bool crash = false;
                for(auto& [x, y] : boatBody){
                    // 为主干道
                    if(mapFunction.isMainChannel(x, y))
                        continue;
                    // 被其他船锁定
                    if(m_boatLock.count({x, y}) && m_boatLock[{x, y}].size() > 1){
                        crash = true;
                        break;
                    }
                }

                if(crash) {
                    continue;
                }

                // 占 berth 小技巧
                if(mapFunction.getNearBerthID(nextBoat[0], nextBoat[1]) >= 0 && mapFunction.getNearBerthID(nextBoat[0], nextBoat[1]) == boats[boatID].berthID){
                    flag = numbers[i];
                    break;
                }

                int depth = mapFunction.getBerthBoatDepth(boat.berthID, nextBoat[0], nextBoat[1], nextBoat[2]);
                if (depth < minDepth) {
                    minDepth = depth;
                    flag = numbers[i];
                }
            }

            if (flag == 0) {
                boat.ship();
            } else if (flag == 1) {
                boat.rot(RotationDirection::Clockwise);
            } else if (flag == 2) {
                boat.rot(RotationDirection::Anticlockwise);
            } else{
                //TODO: 目前直接dept
//                boat.dept();
            }
        }
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // 去delivery船舶寻路
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(boatsGoDelivery.count(boatID)){

            int flag = -1, minDepth = INF;
            for (int i = 0; i < 3; ++i) {

                vector<int> nextBoat;
                if (numbers[i] == 0) {
                    nextBoat = mapFunction.getShip(boat.x, boat.y, boat.dir);
                } else if (numbers[i] == 1) {
                    nextBoat = mapFunction.getClockwise(boat.x, boat.y, boat.dir);
                } else if (numbers[i] == 2) {
                    nextBoat = mapFunction.getAnticlockwise(boat.x, boat.y, boat.dir);
                }

                // 轮船能移动到该位置
                if (!mapFunction.checkBoatCanMoveOn(nextBoat[0], nextBoat[1], nextBoat[2])) {
                    continue;
                }

                // 轮船不会与其他轮船发生碰撞
                auto boatBody = mapFunction.getBoatBody(nextBoat[0], nextBoat[1], nextBoat[2]);
                bool crash = false;
                for(auto& [x, y] : boatBody){
                    // 为主干道
                    if(mapFunction.isMainChannel(x, y))
                        continue;

                    // 其他船位于这片区域 且 该船先移动
                    if(m_boatLock.count({x, y}) && m_boatLock[{x, y}].size() > 1){
                        crash = true;
                        break;
                    }
                }

                if(crash) {
                    continue;
                }

                int depth = mapFunction.getDeliveryDepth(nextBoat[0], nextBoat[1], nextBoat[2]);
                if (depth < minDepth) {
                    minDepth = depth;
                    flag = numbers[i];
                }
            }

            if (flag == 0) {
                boat.ship();
            } else if (flag == 1) {
                boat.rot(RotationDirection::Clockwise);
            } else if (flag == 2) {
                boat.rot(RotationDirection::Anticlockwise);
            } else{
                // TODO: 目前直接dept
                // 在最后时刻dept，可能会出现无法送到交货点的情况
//                boat.dept();
            }
        }
    }
}