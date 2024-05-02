//
// Created by 许晨浩 on 2024/4/16.
//

#include "RobotSchedule.h"
#include <random>

RobotSchedule::RobotSchedule(){
    robotSendWorth = 0;
}

void RobotSchedule::schedule(MapFunction& mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)
    preProcess(mapFunction, globalData);

    // 机器人调度
    optRobots(mapFunction, globalData);
    // 机器人寻路
    moveRobots(mapFunction, globalData);

}

void RobotSchedule::preProcess(MapFunction &mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    //清除上一帧的锁
    m_robotLock.clear();

    //设置这一帧的锁
    for(auto& robot : robots){
        // 锁定机器人的当前位置

        m_robotLock[{robot.x, robot.y}].push_back(robot.id);
        // 初始时不锁定
        if(robot.prevX1 != -1 && robot.prevY1 != -1){
            // 对于前一帧移动的机器人 锁定前进方向的一格
            if(robot.x != robot.prevX1 || robot.y != robot.prevY1){
                int dx = robot.x - robot.prevX1;
                int dy = robot.y - robot.prevY1;

                // 如果可以继续移动，则锁住前进方向
                if(mapFunction.robotCanMoveOn(robot.x + dy, robot.y + dy)){
                    m_robotLock[{robot.x + dx, robot.y + dy}].push_back(robot.id);
                }
                // 如果继续移动会撞墙，则锁住另外两个可移动的方向
                else{
                    // 之前沿着y方向移动，锁x方向
                    if(dx == 0){
                        m_robotLock[{robot.x, robot.y - 1}].push_back(robot.id);
                        m_robotLock[{robot.x, robot.y + 1}].push_back(robot.id);
                    }
                        // 之前沿着x方向移动，锁y方向
                    else{
                        m_robotLock[{robot.x - 1, robot.y}].push_back(robot.id);
                        m_robotLock[{robot.x + 1, robot.y}].push_back(robot.id);
                    }
                }
            }
        }
    }

    //清除上一帧的调度
    robotsTakeGood.clear();
    robotsGoBerth.clear();
    robotsFindGood.clear();
    goodsTaken.clear();
}

void RobotSchedule::optRobots(MapFunction &mapFunction, GlobalData &globalData) {
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)
    // 机器人 get 和 pull
    for (auto& rid : privateRobots) {
        auto& robot = robots[rid];
        if (askedRobots.count(rid)) {// 处于答题状态的机器人不处理
            continue;
        }
        if (robot.goodsNum != robot.type) {
            int gid = mapFunction.getGoodID(robot.x, robot.y);
            // 根据机器人货物的类别来判断, 只要是相同类型的货物就直接 get
            if (gid != -1 && goods[gid].worth > minGoodWorth) {
                if (goods[gid].valuable) {
                    if (!mapFunction.isAnsweredGood(gid)) {
                        // 贵重物品
                        robot.get();
                        robot.targetGoodID = -1;
                        goodsTaken.insert(gid);
                        askedRobots.insert(rid);
                    }

                } else {
                    // 不是贵重物品, 直接拾取
                    robot.get();
                    robot.targetGoodID = -1;
                    goodsTaken.insert(gid);
                    ++robot.realtimeGoodsNum;
                }
                continue;
            }
        }
        if (robot.goodsNum != 0 && mapFunction.getBerthID(robot.x, robot.y) != -1) {
            robot.pull();
            robot.realtimeGoodsNum = 0;
        }
    }

    // 如果其他队伍的一个机器人与货物重合, 我们在本帧假设其已被取走(当前帧); 本队伍的已精准控制, 无需假设
    for (auto& robot : robots) {
        if (!privateRobots.count(robot.id) && robot.goodsNum < robot.type) {
            int gid = mapFunction.getGoodID(robot.x, robot.y);

            if (gid != -1) {
                goodsTaken.insert(gid);
            }

            // 把其他队伍机器人的周围一圈也锁定
            for(auto& [x, y] : moves){
                int u = robot.x + x;
                int v = robot.y + y;

                gid = mapFunction.getGoodID(u, v);

                if (gid != -1) {
                    goodsTaken.insert(gid);
                }
            }

        }
    }

    // TODO: 1.确定好机器人的目标; 2.做好机器人与货物的匹配
    vector<int> robotMatchList;
    for (auto& rid : privateRobots) {
        auto& robot = robots[rid];
        if (askedRobots.count(rid)) {// 处于答题状态的机器人不处理
            continue;
        }
        if (robot.type == robot.realtimeGoodsNum) {
            // 去卸货物
            robotsGoBerth.insert(rid);
        } else {
            // 去取货物
            robotMatchList.push_back(rid);
        }
    }

    cerr << "robotMatchList.size() = " << robotMatchList.size() << endl;

    vector<tuple<double, int, int>> weightRobotGood;
    vector<tuple<int, int, int>> distanceRobotGood;

    for (auto& rid : robotMatchList) {
        // TODO: 为高级机器人一次确定 2 个货物?
        auto& robot = robots[rid];
        vector<tuple<int, int, int>> singleDistanceRobotGood;

        for (auto& [_, gid] : mapFunction.m_pos2goodID) {
            auto& good = goods[gid];
            if (!good.valuable) {
                continue;// 测试: 只考虑高价值货物
            }
            int dist2Good = mapFunction.getGoodRobotDepth(gid, robot.x, robot.y);
            if (dist2Good == INF) {
                continue;
            }
//            cerr << "dist2Good = " << dist2Good << ", robot = {" << robot.x << "," << robot.y << "}, good = {" << good.x << ", " << good.y << "}\n";
            if (mapFunction.isAnsweredGood(gid)) {
                continue;// 已经回答过的高价值货物
            }
            if (!good.isValid(frameID + dist2Good)) {
//                cerr << "dist2Good is too far... dist2Good = " << dist2Good << "\n";
                continue;
            }
            if (goodsTaken.count(gid)) {
                continue;
            }
            int dist2Berth = mapFunction.getBerthRobotDepth(-1, good.x, good.y);
            if (frameID + dist2Good + dist2Berth > maxFrameID) {
//                cerr << "dist2Good + dist2Berth is too far...\n";
                continue;// 取了也来不及送到泊位
            }

            singleDistanceRobotGood.emplace_back(dist2Good, rid, gid);
        }

        sort(singleDistanceRobotGood.begin(), singleDistanceRobotGood.end(), [](auto& a, auto& b) {
            return get<0>(a) < get<0>(b);
        });


        for (int i = 0; i < singleDistanceRobotGood.size() && i < robotMatchList.size(); ++i) {
            distanceRobotGood.push_back(singleDistanceRobotGood[i]);
        }

    }

    sort(distanceRobotGood.begin(), distanceRobotGood.end(), [](auto& a, auto& b) {
        return get<0>(a) < get<0>(b);
    });

    // 按距离贪心匹配
    set<int> usedRobots;
    set<int> usedGoods;
    int valuableGoodRobotCount = 0;
    for(auto& [dist, robotID, goodID] : distanceRobotGood){
        // 已为所有机器人寻找到下一个货物
        if(usedRobots.size() == robotMatchList.size())
            break;

        // 已配对
        if(usedRobots.count(robotID) || usedGoods.count(goodID))
            continue;

        auto& robot = robots[robotID];
        robotsTakeGood.insert(robotID);
        robot.targetGoodID = goodID;
        // 清除先前最远berth的标记
        robot.targetBerthID = -1;

        usedRobots.insert(robotID);
        usedGoods.insert(goodID);
        ++valuableGoodRobotCount;
    }

    // 后处理 1: 如果有机器人携带了 1 个货物且此时没有再匹配到货物, 让它去泊位!
    // 后处理 2: 如果有机器人没有匹配到高价值货物, 那就让它去搜索低价值货物!
    vector<int> robotMatchCheapGoodList;
    for (auto& rid : robotMatchList) {
        if (usedRobots.count(rid)) {
            continue;
        }
        if (robots[rid].realtimeGoodsNum != 0) {
            // 让它去泊位
            robotsGoBerth.insert(rid);
        } else {
            // 去取便宜货物
            robotMatchCheapGoodList.push_back(rid);
        }
    }

    cerr << "robotMatchCheapGoodList.size() = " << robotMatchCheapGoodList.size() << endl;

    vector<tuple<double, int, int>> weightRobotCheapGood;
    for (auto& rid : robotMatchCheapGoodList) {
        auto& robot = robots[rid];
        vector<tuple<double, int, int>> singleWeightRobotGood;
        for (auto& [_, gid] : mapFunction.m_pos2goodID) {
            auto& good = goods[gid];
            if (good.valuable || good.worth <= minGoodWorth) {
                continue;
            }
            if (usedGoods.count(gid)) {
                continue;
            }

            int dist2Good = mapFunction.getGoodRobotDepth(gid, robot.x, robot.y);
            if (dist2Good == INF) {
                continue;
            }
            if (!good.isValid(frameID + dist2Good)) {
                continue;
            }
            int dist2Berth = mapFunction.getBerthRobotDepth(-1, good.x, good.y);
            if (frameID + dist2Good + dist2Berth > maxFrameID) {
                continue;// 取了也来不及送到泊位
            }
            double weight = (double)good.worth / max(dist2Good, 1);
            singleWeightRobotGood.emplace_back(weight, rid, gid);
//            cerr << "w = " << weight << ", rid = " << rid << ", gid = " << gid << endl;
        }

        sort(singleWeightRobotGood.begin(), singleWeightRobotGood.end(), [](auto& a, auto& b) {
            return get<0>(a) > get<0>(b);
        });

//        cerr << "rid = " << rid << ", singleWeightRobotGood.size() = " << singleWeightRobotGood.size() << endl;
        for (int i = 0; i < singleWeightRobotGood.size() && i <= robotMatchCheapGoodList.size(); ++i) {
            weightRobotCheapGood.push_back(singleWeightRobotGood[i]);
        }
    }

    cerr << "weightRobotCheapGood.size() = " << weightRobotCheapGood.size() << endl;

    HungarianAlgorithm alg2(weightRobotCheapGood);
    auto matchedRobotGood2 = alg2.getMatchedRobotAndTarget();
    for (auto& [robotID, goodID] : matchedRobotGood2) {
        auto& robot = robots[robotID];
        robotsTakeGood.insert(robotID);
        robot.targetGoodID = goodID;
        auto& good = goods[goodID];
        usedRobots.insert(robotID);
    }

    for(auto& robotID : robotMatchList){
        // 如果没有匹配成功，去离当前机器人最远的berth
        if(!usedRobots.count(robotID)){
            robotsFindGood.insert(robotID);
        }
    }

    cerr << "matchedRobotGood Valuable = " << valuableGoodRobotCount << endl;
    cerr << "matchedRobotGood Cheap = " << matchedRobotGood2.size() << endl;
}

void RobotSchedule::moveRobots(MapFunction& mapFunction, GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)
    // TODO: 做好机器人与坐标的匹配

//    cerr << "[RobotSchedule]: " << robotsTakeGood.size() << " robots take good\n";
//    cerr << "[RobotSchedule]: " << robotsGoBerth.size() << " robots go berth\n";

    vector<tuple<double, int, int>> weightRobotCoord;
    int coordIdx = 0;
    map<pair<int, int>, int> coord2idx;
    map<int, pair<int, int>> idx2coord;

    auto registerCoord = [&](const int& x, const int& y) {
        if (!coord2idx.count({x, y})) {
            coord2idx[{x, y}] = coordIdx;
            idx2coord[coordIdx] = {x, y};
            ++coordIdx;
        }
    };

    auto getRandomMoveDirs = []() {
        // 随机排序
        vector<int> moveIndices{0, 1, 2, 3};
        random_device rd;
        mt19937 g(rd());
        shuffle(moveIndices.begin(), moveIndices.end(), g);
        return moveIndices;
    };

    // 权重设计:
    // 1. weightFeasible 须足够大, 以保证每个机器人都能被求解器分配坐标
    // 2. weightClose + weightAway > weightKeep * 2
    // 3. 相同距离变动的情况下, 优先将携带货物的机器人推向泊位, 我们通过 weightHighPriority 来实现该功能

    const double weightFeasible = 10336.520f;// 保证每个机器人都有解
    const double weightClose = weightFeasible + 64.0;// 移动, 距离减小
    const double weightKeep = weightFeasible + 16.0;// 移动, 距离不变
    const double weightAway = weightFeasible + 4.0;// 移动, 距离变大
    const double weightStatic = weightFeasible + 1.0;// 不移动, 距离不变
    const double weightHighPriority = 0.2;// 鼓励目标为泊位的机器人优先移动

    set<pair<int, int>> otherRobots;// 其他队伍机器人的位置
    map<pair<int, int>, int> coord2Robot;
    // 1. 为当前所有机器人注册位置
    for (int rid = 0; rid < robots.size(); ++rid) {
        auto& robot = robots[rid];
        coord2Robot[{robot.x, robot.y}] = rid;
        if (privateRobots.count(rid)) {
            // 本队机器人
            registerCoord(robot.x, robot.y);
            weightRobotCoord.emplace_back(weightStatic, rid, coord2idx[{robot.x, robot.y}]);
        } else {
            // 其他队伍的机器人
            // a. 其他队伍机器人下一帧可能的所有位置
            otherRobots.insert({robot.x, robot.y});
            // 暂不考虑其它机器人的寻路
//            // 如果其他队伍的机器人在答题, 直接假设其静止不动
//            int gid = mapFunction.getGoodID(robot.x, robot.y);
//            if (gid != -1 && goods[gid].valuable) {
//                continue;// 高价值货物存在, 则其应该在答题
//            }
//            for (auto& i : getRandomMoveDirs()) {
//                int u = robot.x + moves[i].first;
//                int v = robot.y + moves[i].second;
//                if (mapFunction.robotCanMoveOn(u, v)) {
//                    otherRobots.insert({u, v});
//                }
//            }
        }
    }

    // 命令该机器人去最远的泊位
    auto goFarthestBerth = [&](int rid) {
        auto& robot = robots[rid];
        int farthestBerthID = -1, farthestDist = -1;
        for (int berthID = 0; berthID < berths.size(); ++berthID) {
            int dist = mapFunction.getBerthRobotDepth(berthID, robot.x, robot.y);
            if (dist != INF && dist > farthestDist) {
                farthestDist = dist;
                farthestBerthID = berthID;
            }
        }
        if (farthestBerthID == -1) {
            return;// 无有效泊位
        }

        int currentDepth = mapFunction.getBerthRobotDepth(farthestBerthID, robot.x, robot.y);
        for (auto& i : getRandomMoveDirs()) {
            int u = robot.x + moves[i].first;
            int v = robot.y + moves[i].second;

            if (!mapFunction.robotCanMoveOn(u, v)) {
                continue;// 不可达
            }
            if (!mapFunction.isMainRoad(u, v) && (otherRobots.count({u, v}) || (coord2Robot.count({u, v}) && coord2Robot[{u, v}] > rid))) {
                // 不是主航道就要做避障
                // 1. 其他队伍机器人可能会去
                // 2. 本队伍 ID 靠后的机器人已占据该位置
                continue;
            }
            int nextDepth = mapFunction.getBerthRobotDepth(farthestBerthID, u, v);
//            if (nextDepth == INF) {
//                continue;
//            }
            registerCoord(u, v);
            if (nextDepth < currentDepth) {
                weightRobotCoord.emplace_back(weightClose, robot.id, coord2idx[{u, v}]);
            } else if (nextDepth > currentDepth) {
                weightRobotCoord.emplace_back(weightAway, robot.id, coord2idx[{u, v}]);
            } else {
                weightRobotCoord.emplace_back(weightKeep, robot.id, coord2idx[{u, v}]);
            }
        }

    };

    // 统计我们轮船所在的泊位数据
    unordered_set<int> berthHasOurBoat;
    for (auto& boatID : privateBoats) {
        auto& boat = boats[boatID];
        if (boat.status == 2 || (boat.status == 1 && mapFunction.getBerthID(boat.x, boat.y) != -1)) {
            berthHasOurBoat.insert(boatID);
        }
    }

    // 2. 为本队伍机器人选择下一个坐标
    for (auto& rid : privateRobots) {
        if (askedRobots.count(rid)) {
            continue;// 正在答题的机器人, 我们选择让其静止
        }
        if (!robotsTakeGood.count(rid) && !robotsGoBerth.count(rid)) {
            // 没有匹配到货物的机器人
            goFarthestBerth(rid);
            continue;
        }
//        if (!robotsTakeGood.count(rid) && !robotsGoBerth.count(rid)) {
//            // 应该是没有匹配到货物的机器人 / 正在答题的机器人, 我们选择让其静止
//            continue;
//        }
        auto& robot = robots[rid];
        if (robotsTakeGood.count(rid) && robot.targetGoodID == -1) {
            cerr << "robotsTakeGood.count(rid) && robot.targetGoodID == -1\n";
            continue;// 临时加上, 其实无意义, robotsTakeGood 中的机器人 targetGoodID 应该不为 -1
        }

        if (robotsTakeGood.count(rid)) {
            // a. 去取货物
            int currentDepth = mapFunction.getGoodRobotDepth(robot.targetGoodID, robot.x, robot.y);
            for (auto& i : getRandomMoveDirs()) {
                int u = robot.x + moves[i].first;
                int v = robot.y + moves[i].second;
                if (!mapFunction.robotCanMoveOn(u, v)) {
                    continue;// 不可达
                }
                if (!mapFunction.isMainRoad(u, v) && (otherRobots.count({u, v}) || (coord2Robot.count({u, v}) && coord2Robot[{u, v}] > rid))) {
                    // 不是主航道就要做避障
                    // 1. 其他队伍机器人可能会去
                    // 2. 本队伍 ID 靠后的机器人已占据该位置
                    continue;
                }
                int nextDepth = mapFunction.getGoodRobotDepth(robot.targetGoodID, u, v);
                registerCoord(u, v);
                if (u == robot.prevX1 && v == robot.prevY1) {
                    continue;// 避免在同一个地方晃悠
                }
                if (u == robot.prevX1 && v == robot.prevY1) {
                    continue;// 说明与其它的机器人卡死了
                }
                if (nextDepth < currentDepth) {
                    weightRobotCoord.emplace_back(weightClose, robot.id, coord2idx[{u, v}]);
                } else if (nextDepth > currentDepth) {
                    weightRobotCoord.emplace_back(weightAway, robot.id, coord2idx[{u, v}]);
                } else {
                    weightRobotCoord.emplace_back(weightKeep, robot.id, coord2idx[{u, v}]);
                }
            }
        } else {
            // b. 去泊位卸载货物
            // 选一个泊位出来, 然后送货过去
            int targetBerthID = -1, targetDepth = INF;
            for (int berthID = 0; berthID < berths.size(); ++berthID) {
                auto& berth = berths[berthID];
                int depth = mapFunction.getBerthRobotDepth(berthID, robot.x, robot.y);
                if (depth == INF) {
                    continue;
                }
                // 检查泊位上的货物是否来得及运到交货点
                int loadingFrames = (berth.goodsWorth.size() + berth.loadingSpeed - 1) / berth.loadingSpeed;
                int transportFrames = mapFunction.getMinDistBerth2Delivery(berthID);
                if (frameID + depth + loadingFrames + transportFrames > maxFrameID) {
                    continue;// 运过去也来不及送到交货点
                }
//                if (berthHasOurBoat.count(berthID)) {
//                    depth -= min(depth, distThresholdGoOurBerth);
//                }
                if (depth < targetDepth) {
                    targetDepth = depth;
                    targetBerthID = berthID;
                }
            }

            if (targetBerthID == -1) {
                continue;
            }
            int currentDepth = mapFunction.getBerthRobotDepth(targetBerthID, robot.x, robot.y);
            cerr << "robot[" << rid << "]: go berth[" << targetBerthID << "], dist = " << currentDepth << "\n";
            for (auto& i : getRandomMoveDirs()) {
                int u = robot.x + moves[i].first;
                int v = robot.y + moves[i].second;
                if (!mapFunction.robotCanMoveOn(u, v)) {
                    continue;// 不可达
                }
                if (!mapFunction.isMainRoad(u, v) && (otherRobots.count({u, v}) || (coord2Robot.count({u, v}) && coord2Robot[{u, v}] > rid))) {
                    // 不是主航道就要做避障
                    // 1. 其他队伍机器人可能会去
                    // 2. 本队伍 ID 靠后的机器人已占据该位置
                    continue;
                }
                int nextDepth = mapFunction.getBerthRobotDepth(targetBerthID, u, v);
                registerCoord(u, v);
                if (u == robot.prevX1 && v == robot.prevY1) {
                    continue;// 避免在同一个地方晃悠
                }
                if (u == robot.prevX1 && v == robot.prevY1) {
                    continue;// 说明与其它的机器人卡死了
                }
                if (nextDepth < currentDepth) {
                    weightRobotCoord.emplace_back(weightClose, robot.id, coord2idx[{u, v}]);
                } else if (nextDepth > currentDepth) {
                    weightRobotCoord.emplace_back(weightAway, robot.id, coord2idx[{u, v}]);
                } else {
                    weightRobotCoord.emplace_back(weightKeep, robot.id, coord2idx[{u, v}]);
                }
            }
        }
    }

    // 求解所有机器人的位置
    HungarianAlgorithm algRobotCoord(weightRobotCoord);
    auto matchedRobotCoord = algRobotCoord.getMatchedRobotAndTarget();
//    cerr << "matchedRobotCoord.size() = " << matchedRobotCoord.size() << endl;
    for (auto& [rid, idx] : matchedRobotCoord) {
        auto& robot = robots[rid];
        int srcX = robot.x, srcY = robot.y;
        auto [nextX, nextY] = idx2coord[idx];
        robot.wantX1 = nextX, robot.wantY1 = nextY;
        // 允许机器人不动
        if (nextX - srcX == 1) {
            robot.move((int)MoveDirection::Down);
        } else if (nextX - srcX == -1) {
            robot.move((int)MoveDirection::Up);
        } else if (nextY - srcY == 1) {
            robot.move((int)MoveDirection::Right);
        } else if (nextY - srcY == -1) {
            robot.move((int)MoveDirection::Left);
        } else {
            cerr << "static robot = " << rid << endl;
        }
    }
}