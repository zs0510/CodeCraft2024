#include "Schedule.h"

void Schedule::optimize(int frameID,
                        vector<Robot>& robots,
                        vector<Good>& goods,
                        vector<Berth>& berths,
                        vector<Boat>& boats,
                        AStar* aStar,
                        int& goodsLeftPtr) {

    // 在一帧中, 会按照如下顺序结算
    // 1. 机器人恢复
    // 2. 轮船到达, 进入港口
    // 3. 物品生成
    // 4. 生成场面信息, 选手获取  ***** input() *****
    // 5. 读取选手指令          ***** output() *****
    // 6. 执行机器人指令
    // 7. 执行轮船指令
    // 8. 港口装卸货物

//    if (frameID > 1000) return;

    // 本调度执行顺序
//    cerr << "[Schedule]: frame id = " << frameID << endl;

    ++m_actualFrameCount;

    while (goodsLeftPtr < goods.size() && !goods[goodsLeftPtr].isValid(frameID)) {
        //清除货物的深度图
        goods[goodsLeftPtr].depthMap.clear();
        ++goodsLeftPtr;
    }

    // 根据本帧信息与上一帧信息, 进行仿真
//    simulate(frameID, robots, goods, berths, boats);

    // 在帧 ID 达到某帧时, 屏蔽部分泊位, 减小在泊位上的浪费
    initIgnoreBerths(frameID, robots, berths, boats);

//    updateLockMap(frameID, robots, goods, berths, boats, aStar, goodsLeftPtr);

    for (auto& robot : robots) {
        //上一帧把货物pull的机器人reset
        if(robot.prevGoodStatus == 1 && robot.goodStatus == 0)
            robot.reset();

        //货物送设置为被携带
        if(robot.prevGoodStatus == 0 && robot.goodStatus == 1)
            goods[robot.goodID].status = 2;
    }


    // 1. 优化未携带货物的调度
    optRobotsWithoutGoodDebug(frameID, robots, goods, berths, boats, aStar, goodsLeftPtr);

    // 2. 优化携带货物的调度
    optRobotsWithGood(frameID, robots, goods, berths, boats, aStar, goodsLeftPtr);

    // 3. 移动机器人
    moveRobots(frameID, robots, goods, berths);

    // 3. 优化轮船的调度
    optBoats(frameID, robots, goods, berths, boats, aStar, goodsLeftPtr);

    // 秘籍: 左右横跳解除
//    untangle(robots);

    for (auto& robot : robots) {
        robot.prevGoodStatus = robot.goodStatus;
        robot.prevGoodID = robot.goodID;
        robot.prevX = robot.x;
        robot.prevY = robot.y;
    }

}

void Schedule::identifyMapID() {

    m_mapID = 3;

    int m_barrierCount = 0;
    for (int i = 0; i < m_map.size(); ++i) {
        for (auto& ch : m_map[i]) {
            if (ch == '#') {
                ++m_barrierCount;
            }
        }
    }

    if (m_barrierCount == 0) {
        m_mapID = 1;// 无障碍
        robotEpsilonFrame = robotEpsilonFrame1;
    } else if (m_barrierCount == 10152) {
        m_mapID = 2;// 迷宫
        robotEpsilonFrame = robotEpsilonFrame2;
    } else{
        m_mapID = 3;//隐藏图
        robotEpsilonFrame = robotEpsilonFrame3;
    }

    cerr << "*********************\t Information of Map \t****************************" << endl;
    cerr << "count of barriers = " << m_barrierCount << endl;
    cerr << "map id = " << m_mapID << endl << endl;

}

void Schedule::optRobotsWithoutGoodDebug(int frameID, vector<Robot> &robots, vector<Good> &goods, vector<Berth> &berths,
                                         vector<Boat> &boats, AStar *aStar, int &goodsLeftPtr) {
    // 本帧仍未取得货物的机器人集合
    vector<int> robotsWithoutGood;

    for (auto& robot : robots) {
        // 被隔绝的机器人不处理
        if (m_partition[robot.x][robot.y] == -1) continue;

        // 恢复状态的机器人不处理
        if (robot.moveStatus == 0) continue;

        // 携带货物的机器人不处理
        if (robot.goodStatus == 1) continue;

        // 检查有没有到达目标货物位置
        if (robot.goodID != -1 && robot.goodID < goods.size() && goods[robot.goodID].isValid(frameID)) {
            if (robot.x == goods[robot.goodID].x && robot.y == goods[robot.goodID].y) {
                robot.get();
                goods[robot.goodID].status = 2;// 标记为被携带, 禁止其它机器人抢走货物
                continue;
            }
        }
        robotsWithoutGood.push_back(robot.id);
    }

    vector<tuple<double, int, int>> totalTuples;// get<0> 权重, get<1> 机器人 ID, get<2> 货物 ID
    for (auto& robotID : robotsWithoutGood) {
        int robotX = robots[robotID].x, robotY = robots[robotID].y;
        for (int gid = goodsLeftPtr; gid < goods.size(); ++gid) {
            int goodX = goods[gid].x, goodY = goods[gid].y;
            // 货物已被取走, 货物不与机器人在同一连通分量
            if (goods[gid].status == 2 || m_partition[robotX][robotY] != m_partition[goodX][goodY]) continue;
            int sid = m_pos2Square[goodX][goodY];
            double estimateDist = m_squareDepth[sid][m_pos2SquareConnect[goodX][goodY]][robotX][robotY];

            int squareCenterX = m_squareConnectCenter[sid][m_pos2SquareConnect[goodX][goodY]].first;
            int squareCenterY = m_squareConnectCenter[sid][m_pos2SquareConnect[goodX][goodY]].second;

            //识别出map1和2
            if(m_mapID == 1){
                //直接使用估计距离

                // 来不及取
                if (!goods[gid].isValid(frameID + estimateDist + m_squareDepth[sid][m_pos2SquareConnect[goodX][goodY]][goodX][goodY] + robotEpsilonFrame)) {
                    continue;
                }

                int berthID, minBerthDist = INF;
                for(int i = 0; i < numOfBerths; i++){
                    if(m_ignoreBerths.count(i))
                        continue;
                    if(minBerthDist > m_berthDepth[i][robotX][robotY]){
                        berthID = i;
                        minBerthDist = m_berthDepth[i][robotX][robotY];
                    }
                }

                if(minBerthDist == INF){
                    for(int i = 0; i < numOfBerths; i++){
                        if(minBerthDist > m_berthDepth[i][robotX][robotY]){
                            berthID = i;
                            minBerthDist = m_berthDepth[i][robotX][robotY];
                        }
                    }
                }

                // TODO: 如何优化权重?
                double weight = goods[gid].worth / (estimateDist + fmax(m_berthDepth[berthID][robotX][robotY], 1));
                if (gid == robots[robotID].goodID) {
                    //深情系数
                    weight *= deepLoveCoefficient1;
                }
                totalTuples.push_back(tuple<double, int, int>(weight, robotID, gid));
            }
            else if(m_mapID == 2){
                //直接使用估计距离

                // 来不及取
                if (!goods[gid].isValid(frameID + estimateDist + m_squareDepth[sid][m_pos2SquareConnect[goodX][goodY]][goodX][goodY] + robotEpsilonFrame)) {
                    continue;
                }

                int berthID, minBerthDist = INF;
                for(int i = 0; i < numOfBerths; i++){
                    if(m_ignoreBerths.count(i))
                        continue;
                    if(minBerthDist > m_berthDepth[i][robotX][robotY]){
                        berthID = i;
                        minBerthDist = m_berthDepth[i][robotX][robotY];
                    }
                }

                if(minBerthDist == INF){
                    for(int i = 0; i < numOfBerths; i++){
                        if(minBerthDist > m_berthDepth[i][robotX][robotY]){
                            berthID = i;
                            minBerthDist = m_berthDepth[i][robotX][robotY];
                        }
                    }
                }

                // TODO: 如何优化权重?
                double weight = goods[gid].worth / (estimateDist + fmax(m_berthDepth[berthID][robotX][robotY], 1));
                if (gid == robots[robotID].goodID) {
                    //深情系数
                    weight *= deepLoveCoefficient2;
                }
                totalTuples.push_back(tuple<double, int, int>(weight, robotID, gid));
            }
            else if(m_mapID == 3){

                //robot到good的曼哈顿距离
                double ManhattanDist1 = abs(robotX-goodX) + abs(robotY-goodY);
                //robot到联通分量中心的曼哈顿距离
                double ManhattanDist2 = abs(robotX-squareCenterX) + abs(robotY-squareCenterY);

                if(ManhattanDist2 <= m_squareLength){
                    estimateDist = ManhattanDist1;
                }
                else if(ManhattanDist2 <= 2*m_squareLength){
                    estimateDist = (estimateDist+ManhattanDist1) / 2;
                }
                else{
                    estimateDist = estimateDist * (ManhattanDist1/ManhattanDist2);
                }

                // 来不及取
                if (!goods[gid].isValid(frameID + estimateDist + m_squareDepth[sid][m_pos2SquareConnect[goodX][goodY]][goodX][goodY] + robotEpsilonFrame)) {
                    continue;
                }

                int berthID, minBerthDist = INF;
                for(int i = 0; i < numOfBerths; i++){
                    if(m_ignoreBerths.count(i))
                        continue;
                    if(minBerthDist > m_berthDepth[i][robotX][robotY]){
                        berthID = i;
                        minBerthDist = m_berthDepth[i][robotX][robotY];
                    }
                }

                if(minBerthDist == INF){
                    for(int i = 0; i < numOfBerths; i++){
                        if(minBerthDist > m_berthDepth[i][robotX][robotY]){
                            berthID = i;
                            minBerthDist = m_berthDepth[i][robotX][robotY];
                        }
                    }
                }

                // TODO: 如何优化权重?
                double weight = goods[gid].worth / (estimateDist + fmax(m_berthDepth[berthID][robotX][robotY], 1));
                if (gid == robots[robotID].goodID) {
                    //特调深情系数
                    weight *= deepLoveCoefficient3;
                }
                totalTuples.push_back(tuple<double, int, int>(weight, robotID, gid));

            }

        }
    }

    // TODO: 动态规划? 让机器人与货物总的匹配度最佳
    sort(totalTuples.begin(), totalTuples.end(), [](tuple<double, int, int>& t1, tuple<double, int, int>& t2) {
        return get<0>(t1) > get<0>(t2);
    });

    if(!hungarian){
        // 策略 1: 贪心算法
        unordered_set<int> usedRobots, usedGoods;
        vector<pair<int, int>> robotGoodsPairList;
        for (auto& [w, robotID, goodID] : totalTuples) {
            if (usedRobots.size() == robotsWithoutGood.size()) {
                // 已为所有机器人寻找到下一个货物
                break;
            }
            // 已配对
            if (usedRobots.count(robotID) || usedGoods.count(goodID)) {
                continue;
            }
            auto& robot = robots[robotID];

            usedRobots.insert(robotID);
            usedGoods.insert(goodID);
            robot.goodID = goodID;
        }
    }
    else{
        // 策略 2: 匈牙利算法
        vector<tuple<double, int, int>> hungarianTuples;
        unordered_map<int, int> rid2TupleNum;
        for (auto& [w, rid, gid] : totalTuples) {
            if (!rid2TupleNum.count(rid)) {
                rid2TupleNum[rid] = 0;
            }
            // 每个机器人最多只需要存当前机器人个数的最佳解即可
            if (rid2TupleNum[rid] <= robotsWithoutGood.size()) {
                ++rid2TupleNum[rid];
                hungarianTuples.emplace_back(w, rid, gid);
            }
        }

        unordered_set<int> usedRobots, usedGoods;
        vector<pair<int, int>> robotGoodsPairList;

        HungarianAlgorithm alg(hungarianTuples);
        auto matchedRobotGood = alg.getMatchedRobotAndTarget();
        for (auto& [robotID, goodID] : matchedRobotGood) {
            if (usedRobots.size() == robotsWithoutGood.size()) {
                // 已为所有机器人寻找到下一个货物
                break;
            }
            // 已配对
            if (usedRobots.count(robotID) || usedGoods.count(goodID)) {
                continue;
            }
            auto& robot = robots[robotID];

            usedRobots.insert(robotID);
            usedGoods.insert(goodID);
            robot.goodID = goodID;
        }
    }

}

void Schedule::optRobotsWithGood(int frameID, vector<Robot> &robots, vector<Good> &goods, vector<Berth> &berths,
                                 vector<Boat> &boats, AStar *aStar, int &goodsLeftPtr) {

    // 2. 携带了货物的机器人
    // TODO: 选择泊位时, 考虑泊位去虚拟点的时间
    // TODO: 选择泊位时, 考虑泊位的装卸速度
    // TODO: 在 帧ID 较大时, 考虑禁止一些泊位

    //记录送货机器人
    vector<pair<int, int>> sendRobots;

    for (auto& robot : robots) {
        // 被隔绝的机器人不处理
        if (m_partition[robot.x][robot.y] == -1) continue;

        // 恢复状态的机器人不处理
        if (robot.moveStatus == 0) continue;

        // 只处理携带货物的机器人
        if (robot.goodStatus == 0) continue;

        // 到达港口后卸货
        if (m_map[robot.x][robot.y] == 'B') {
            int berthID = m_pos2Berth[{robot.x, robot.y}];
            if(berths[berthID].transportTime+frameID <= maxFrameID){
                berths[berthID].goodsWorth.push(goods[robot.goodID].worth);
                berths[berthID].sumOfGoodsWorth += goods[robot.goodID].worth;
            }
            m_scoreOfRobot2Berth += goods[robot.goodID].worth;
            m_scoreOfBerth[berthID] += goods[robot.goodID].worth;

            robot.pull();
            robot.reset();
            continue;
        }

        int robotX = robot.x, robotY = robot.y;
        int berthID, minBerthDist = INF;
        for(int i = 0; i < numOfBerths; i++){
            if(m_ignoreBerths.count(i))
                continue;
            if(minBerthDist > m_berthDepth[i][robotX][robotY]){
                berthID = i;
                minBerthDist = m_berthDepth[i][robotX][robotY];
            }
        }

        if(minBerthDist == INF){
            for(int i = 0; i < numOfBerths; i++){
                if(minBerthDist > m_berthDepth[i][robotX][robotY]){
                    berthID = i;
                    minBerthDist = m_berthDepth[i][robotX][robotY];
                }
            }
        }

        goods[robot.goodID].berthID = berthID;

    }

}

void Schedule::optBoats(int frameID, vector<Robot> &robots, vector<Good> &goods, vector<Berth> &berths,
                        vector<Boat> &boats, AStar *aStar, int &goodsLeftPtr) {

    // 3. 轮船调度

    // 优先处理较空的轮船
    vector<int> boatIDs(numOfBoats);
    for (int i = 0; i < numOfBoats; ++i) {
        boatIDs[i] = i;
    }
    sort(boatIDs.begin(), boatIDs.end(), [&](int bid1, int bid2) {
        return boats[bid1].capacity - boats[bid1].size > boats[bid2].capacity - boats[bid2].size;
    });

    // 优先处理较富余的泊位
    vector<int> berthIDs(numOfBerths);
    for (int i = 0; i < numOfBerths; ++i) {
        berthIDs[i] = i;
    }
    sort(berthIDs.begin(), berthIDs.end(), [&](int bid1, int bid2) {
        return berths[bid1].sumOfGoodsWorth > berths[bid2].sumOfGoodsWorth;
    });

    // 对每一个轮船下达指令
    for (auto& boatID : boatIDs) {
        auto& boat = boats[boatID];
        // 已验证, 状态 0, 1, 2 的轮船均可以接受指令
        if (boat.berthID != -1) {
            // 基本约束 1: 需要在 15000 帧前到达虚拟点
            int t = berths[boat.berthID].transportTime;
            if (boat.status == 0 && boat.prevBerthID != -1) {
                // 移动中的轮船, 其与虚拟点的运输时间为上个泊位的运输时间
                // 注意: 仅考虑前一个泊位的运输时间不能保证及时运输回虚拟点
                //     例如 轮船i 从 泊位a(transportTime = 500) 移动到 泊位b(transportTime = 1000), 在到达 泊位b 后回虚拟点的时间会暴涨 500
                t = max(t, berths[boat.prevBerthID].transportTime);
            }
            if (frameID + t + boatEpsilonFrame >= maxFrameID) {
                boat.go(frameID);
                boat.berthID = -1;// 让其它轮船知道其去虚拟点
                continue;
            }
            // 基本约束 2: 船满即走
            if (boat.size == boat.capacity) {
                boat.go(frameID);
                boat.berthID = -1;// 让其它轮船知道其去虚拟点
                continue;
            }
        }

        // 根据船的状态来分别指令
        switch (boat.status) {
            case 0:// 轮船在移动中
            {
                if (boat.berthID == -1) {
//                    // 正在赶往虚拟点
//                    // TODO: 检查有无货物较少的泊位, 本轮船可顺路载其货物去虚拟点
//                    for (auto& i : berthIDs) {
////                            if (i == boat.berthID || berths[i].sumOfGoodsWorth < 1500) continue;
//                        if (i == boat.berthID) continue;
//                        // 已有轮船
//                        if (existSufficientBoat(berths[i], boats, boatID)) continue;
//                        int transportTime = framesFromBerth2Berth + berths[i].transportTime;
//                        int loadingTime = min((int)berths[i].goodsWorth.size() + berths[i].loadingSpeed - 1, boat.capacity - boat.size)
//                                          / berths[i].loadingSpeed * 0.2;
//                        if (frameID + transportTime + loadingTime + epsilonFrame < maxFrameID) {
//                            boat.ship(i, frameID);
//                            boat.berthID = i;
//                            break;
//                        }
//                    }
                } else {
                    // 正在赶往泊位
                    // TODO: 检查有无货物更足的泊位

                }
            }
                break;
            case 1:// 轮船正常状态
            {
                if (boat.berthID != -1) {
                    // 轮船在泊位上
                    auto& berth = berths[boat.berthID];
                    // 模拟载货
                    int loadingCnt = 0;
                    while (loadingCnt < berth.loadingSpeed && !berth.goodsWorth.empty() && boat.size < boat.capacity) {
                        int worth = berth.goodsWorth.front();
                        berth.goodsWorth.pop();
                        berth.sumOfGoodsWorth -= worth;
                        ++boat.size;
                        boat.worth += worth;
                        ++loadingCnt;
                    }
                    // 注意: 根据任务书, 轮船的指令会在泊位装卸货物之前, 如果对当前帧装载了货物的机器人下达指令, 会出现统计错误(该次载货无效)
                    // 仅在当前泊位空的时候才考虑移走轮船
                    if (loadingCnt == 0) {
                        // 泊位点空, 考虑是否去其它泊位 / 虚拟点
                        // 首先检查能否去其它泊位带点货物走
                        bool goToOtherBerth = false;
                        for (auto& i : berthIDs) {
//                            if (i == boat.berthID || berths[i].sumOfGoodsWorth < 1500) continue;
                            if (i == boat.berthID) continue;
                            // 已有轮船
                            if (existSufficientBoat(berths[i], boats, boatID)) continue;
                            int transportTime = framesFromBerth2Berth + berths[i].transportTime;
                            int loadingTime = min((int)berths[i].goodsWorth.size() + berths[i].loadingSpeed - 1, boat.capacity - boat.size)
                                              / berths[i].loadingSpeed * boatCoff;
                            if (frameID + transportTime + loadingTime + boatEpsilonFrame < maxFrameID) {
                                goToOtherBerth = true;
                                boat.ship(i, frameID);
                                boat.berthID = i;
                                break;
                            }
                        }

                        int minBerthTransportTime = INF;
                        for (int i = 0; i < numOfBerths; ++i) {
                            minBerthTransportTime = min(minBerthTransportTime, berths[i].transportTime);
                        }
                        if (berth.transportTime + minBerthTransportTime + boatEpsilonFrame >  maxFrameID) {
                            continue;
                        }

                        // 如果没有合适的泊位就去虚拟点
                        if (!goToOtherBerth) {
                            boat.go(frameID);
                            boat.berthID = -1;
                        }
                    }
                } else {
                    // 在虚拟点, 卸货
                    m_scoreOfBoat2Virtual += boat.worth;
                    boat.reset();
                    // 考虑将其移至其它泊位
                    for (auto& i : berthIDs) {
                        if (existSufficientBoat(berths[i], boats, boatID)) continue;
                        int transportTime = berths[i].transportTime;
                        int loadingTime = (berths[i].goodsWorth.size() + berths[i].loadingSpeed - 1)
                                          / berths[i].loadingSpeed * boatCoff;
                        if (transportTime + loadingTime + boatEpsilonFrame < maxFrameID) {
                            boat.ship(i, frameID);
                            boat.berthID = i;
                            break;
                        }
                    }
                }
            }
                break;
            case 2:// 轮船等待状态
            {
                // 考虑将其移至其它泊位
                for (auto& i : berthIDs) {
                    if (i == boat.berthID || existSufficientBoat(berths[i], boats, boatID)) continue;
                    int transportTime = berths[i].transportTime;
                    int loadingTime = (berths[i].goodsWorth.size() + berths[i].loadingSpeed - 1)
                                      / berths[i].loadingSpeed * boatCoff;
                    if (framesFromBerth2Berth + transportTime + loadingTime + boatEpsilonFrame < maxFrameID) {
                        boat.ship(i, frameID);
                        boat.berthID = i;
                        break;
                    }
                }
            }
                break;
        }

    }

}

void Schedule::initPartition(const vector<Berth>& berths) {

    int n = m_map.size(), m = m_map[0].size();
    m_partition = vector<vector<int>>(n, vector<int>(m, -1));
    int partitionID = 0;
    for (auto& berth : berths) {
        int srcX = berth.x;
        int srcY = berth.y;

        // 已被划入分区
        if (m_partition[srcX][srcY] != -1) continue;

        queue<pair<int, int>> que;
        que.push({srcX, srcY});
        m_partition[srcX][srcY] = partitionID;
        while (!que.empty()) {
            auto [x, y] = que.front();
            que.pop();
            for (auto& [dx, dy] : moves) {
                int u = x + dx;
                int v = y + dy;
                if (canMoveOn(u, v) && m_partition[u][v] == -1) {
                    que.push({u, v});
                    m_partition[u][v] = partitionID;
                }
            }
        }
        ++partitionID;
    }
    cerr << "[Schedule]: we get " << partitionID << " partition(s)." << endl;
}

void Schedule::initBerthDepth(const vector<Berth> &berths) {

    int n = m_map.size(), m = m_map[0].size();
    m_berthDepth = vector<vector<vector<int>>>(numOfBerths, vector<vector<int>>(n, vector<int>(m, INF)));
    for (int berthID = 0; berthID < numOfBerths; ++berthID) {
        auto& berth = berths[berthID];
        auto& berthDepthMap = m_berthDepth[berthID];
        queue<pair<pair<int, int>, int>> que;// [x, y, depth]
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (canMoveOn(berth.x + i, berth.y + j)) {
                    berthDepthMap[berth.x + i][berth.y + j] = 0;
                    que.push({pair<int, int>(berth.x + i, berth.y + j), 0});
                    m_map[berth.x + i][berth.y + j] = 'B';
                    m_pos2Berth[{berth.x + i, berth.y + j}] = berthID;
                }
            }
        }
        // 广搜扩散
        while (!que.empty()) {
            auto [pos, depth] = que.front();
            que.pop();
            auto [x, y] = pos;
            for (auto& [dx, dy] : moves) {
                int u = x + dx;
                int v = y + dy;
                if (canMoveOn(u, v) && berthDepthMap[u][v] > depth + 1) {
                    berthDepthMap[u][v] = depth + 1;
                    que.push({pair<int, int>(u, v), depth + 1});
                }
            }
        }
//        cerr << "**************** It is a depth map!!! ****************" << endl;
//        for (int i = 0; i < n; ++i) {
//            for (int j = 0; j < m; ++j) {
//                if (berthDepthMap[i][j] == INF) {
//                    cerr << "*";
//                } else {
//                    cerr << (berthDepthMap[i][j] % 10);
//                }
//            }
//            cerr << endl;
//        }
//        cerr << endl;
    }

    m_nearestBerthDist = vector<vector<int>>(n, vector<int>(m, INF));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            for (int berthID = 0; berthID < numOfBerths; ++berthID) {
                m_nearestBerthDist[i][j] = min(m_nearestBerthDist[i][j], m_berthDepth[berthID][i][j]);
            }
        }
    }

}

//更新部分
void Schedule::moveRobots(int frameID,
                vector<Robot>& robots,
                vector<Good>& goods,
                vector<Berth>& berths){

    //清除锁
    m_lockMap.clear();
    m_directionLockMap.clear();

    set<int> moveRobotSet;

    for(auto& robot : robots){
//        //机器人位于恢复状态
        if(robot.moveStatus == 0){
            //锁定当前位置
            m_lockMap.insert({robot.x, robot.y});
            //加入moveRobotSet中
            moveRobotSet.insert(robot.id);
        }
//        m_lockMap.insert({robot.x, robot.y});
    }

    while(moveRobotSet.size() < numOfRobots){
        //越低的机器人优先寻路
        vector<pair<int, int>> robotPriorityList;

        for(auto& robot : robots){
            //当前机器人已寻路
            if(moveRobotSet.count(robot.id)) {
                continue;
            }

            int priority;
            //带货机器人
            if(robot.goodStatus == 0){
                //有目标
                if(robot.goodID >= 0){
                    //可以通过修改count改变取货机器人的优先级别
                    priority = takingRobotsPriority;
                }
                //没有目标 优先级最低
                else{
                    priority = freeRobotsPriority;
                }
            }
            //送货机器人
            else{
                //可以通过修改count改变取货机器人的优先级别
                priority = sendingRobotsPriority;
            }

            //统计机器人的可移动方向
            int count = 0;

            int robotX = robot.x;
            int robotY = robot.y;

            for(int moveID = 0; moveID < 4; moveID++){
                int u = robotX + moves[moveID].first;
                int v = robotY + moves[moveID].second;

                if(canMoveOn(u, v) && !m_lockMap.count({u, v})){
                    //判断方向锁
                    if(m_directionLockMap.count({robotX, robotY}) && m_directionLockMap[{robotX, robotY}] == moveID)
                        continue;
                    count++;
                }
            }

            priority += count*3;
            robotPriorityList.push_back({robot.id, priority});
        }

        sort(robotPriorityList.begin(), robotPriorityList.end(), [](pair<int, int>& p1, pair<int, int>& p2) {
            return p1.second < p2.second;
        });

        auto& robot = robots[robotPriorityList.front().first];
        //加入moveRobotSet中
        moveRobotSet.insert(robot.id);

        int robotX = robot.x;
        int robotY = robot.y;

        //带货机器人
        if(robot.goodStatus == 0){
            //有目标
            if(robot.goodID >= 0){
                int targetX = goods[robot.goodID].x;
                int targetY = goods[robot.goodID].y;

                int sid = m_pos2Square[targetX][targetY];
                int scid = m_pos2SquareConnect[targetX][targetY];

                int estimateDistance = m_squareDepth[sid][scid][robotX][robotY];

                if(estimateDistance <= 2*m_squareLength){
                    //初始化深度图
                    if(goods[robot.goodID].depthMap.size() == 0){
                        goods[robot.goodID].depthMap = vector<vector<int>> (szOfMap+1, vector<int> (szOfMap+1, INF));

                        queue<pair<pair<int, int>, int>> que;// [x, y, depth]
                        goods[robot.goodID].depthMap[targetX][targetY] = 0;
                        que.push({pair<int, int>(targetX, targetY), 0});

                        // 广搜扩散
                        while (!que.empty()) {
                            auto [pos, depth] = que.front();
                            que.pop();
                            //不需要继续传播
                            if(depth > 2*(m_squareLength+5))
                                continue;

                            auto [x, y] = pos;
                            for (auto& [dx, dy] : moves) {
                                int u = x + dx;
                                int v = y + dy;
                                if (canMoveOn(u, v) && goods[robot.goodID].depthMap[u][v] > depth + 1) {
                                    goods[robot.goodID].depthMap[u][v] = depth + 1;
                                    que.push({pair<int, int>(u, v), depth + 1});
                                }
                            }
                        }

                    }
                }

                //商品深度图初始化完成且深度图上的距离有效 按照深度图走
                if(goods[robot.goodID].depthMap.size() > 0 && goods[robot.goodID].depthMap[robotX][robotY] != INF){
                    int moveID = -1, minDist = INF;
//                    //能否留在原地
//                    if(!m_lockMap.count({robotX, robotY})){
//                        minDist = goods[robot.goodID].depthMap[robotX][robotY];
//                    }

                    //是否需要移动
                    for(int i = 0; i < 4; i++){
                        int u = robotX + moves[i].first;
                        int v = robotY + moves[i].second;

                        if(canMoveOn(u, v) && !m_lockMap.count({u, v})){
                            //判断方向锁
                            if(m_directionLockMap.count({robotX, robotY}) && m_directionLockMap[{robotX, robotY}] == i)
                                continue;

                            int temp = goods[robot.goodID].depthMap[u][v];
                            if(temp == INF){
                                temp = m_squareDepth[sid][scid][u][v];
                            }
                            if(temp < minDist){
                                minDist = goods[robot.goodID].depthMap[u][v];
                                moveID = i;
                            }
                        }
                    }

                    // 对机器人发出指令, 并锁定下一帧的位置
                    if (moveID == -1) {
                        //不移动 锁定当前位置
                        m_lockMap.insert({robotX, robotY});
//                        cerr << "We cannot find a feasible next grid for robot" + to_string(robot.id) + " without goods!!!" << endl;
                    }
                    else {
                        int u = robot.x + moves[moveID].first;
                        int v = robot.y + moves[moveID].second;

                        m_lockMap.insert({u, v});

                        //方向锁
                        if(moveID == 0){
                            m_directionLockMap[{u, v}] = 1;
                        }
                        else if(moveID == 1){
                            m_directionLockMap[{u, v}] = 0;
                        }
                        else if(moveID == 2){
                            m_directionLockMap[{u, v}] = 3;
                        }
                        else if(moveID == 3){
                            m_directionLockMap[{u, v}] = 2;
                        }

                        robot.move(directionMap[moveID]);

                        if(u == targetX && v == targetY){
                            robot.get();
                        }
                    }
                }
                else{
                    //由分区引导
                    int moveID = -1, minDist = INF;
//                    //能否留在原地
//                    if(!m_lockMap.count({robotX, robotY})){
//                        minDist = m_squareDepth[sid][scid][robotX][robotY];
//                    }

                    //是否需要移动
                    for(int i = 0; i < 4; i++){
                        int u = robotX + moves[i].first;
                        int v = robotY + moves[i].second;

                        if(canMoveOn(u, v) && !m_lockMap.count({u, v})){
                            //判断方向锁
                            if(m_directionLockMap.count({robotX, robotY}) && m_directionLockMap[{robotX, robotY}] == i)
                                continue;

                            if(m_squareDepth[sid][scid][u][v] < minDist){
                                minDist = m_squareDepth[sid][scid][u][v];
                                moveID = i;
                            }
                        }
                    }

                    // 对机器人发出指令, 并锁定下一帧的位置
                    if (moveID == -1) {
                        //不移动 锁定当前位置
                        m_lockMap.insert({robotX, robotY});
//                        cerr << "We cannot find a feasible next grid for robot" + to_string(robot.id) + " without goods!!!" << endl;
                    }
                    else {
                        int u = robot.x + moves[moveID].first;
                        int v = robot.y + moves[moveID].second;

                        m_lockMap.insert({u, v});

                        //方向锁
                        if(moveID == 0){
                            m_directionLockMap[{u, v}] = 1;
                        }
                        else if(moveID == 1){
                            m_directionLockMap[{u, v}] = 0;
                        }
                        else if(moveID == 2){
                            m_directionLockMap[{u, v}] = 3;
                        }
                        else if(moveID == 3){
                            m_directionLockMap[{u, v}] = 2;
                        }

                        robot.move(directionMap[moveID]);
                    }
                }
            }
            //没有目标
            else{
                //之前有目标
                if(robot.prevGoodID >= 0){
                    int moveID = -1, minDist = INF;

                    //是否需要移动
                    for(int i = 0; i < 4; i++){
                        int u = robotX + moves[i].first;
                        int v = robotY + moves[i].second;

                        if(canMoveOn(u, v) && !m_lockMap.count({u, v})){
                            //判断方向锁
                            if(m_directionLockMap.count({robotX, robotY}) && m_directionLockMap[{robotX, robotY}] == i)
                                continue;

                            int manhattanDist = abs(u-goods[robot.prevGoodID].x) + abs(v-goods[robot.prevGoodID].y);
                            if(manhattanDist < minDist){
                                manhattanDist = minDist;
                                moveID = i;
                            }

                        }
                    }

                    // 对机器人发出指令, 并锁定下一帧的位置
                    if (moveID == -1) {
                        //不移动 锁定当前位置
                        m_lockMap.insert({robotX, robotY});
//                        cerr << "We cannot find a feasible next grid for robot" + to_string(robot.id) + " without goods!!!" << endl;
                    }
                    else {
                        int u = robot.x + moves[moveID].first;
                        int v = robot.y + moves[moveID].second;

                        m_lockMap.insert({u, v});

                        //方向锁
                        if(moveID == 0){
                            m_directionLockMap[{u, v}] = 1;
                        }
                        else if(moveID == 1){
                            m_directionLockMap[{u, v}] = 0;
                        }
                        else if(moveID == 2){
                            m_directionLockMap[{u, v}] = 3;
                        }
                        else if(moveID == 3){
                            m_directionLockMap[{u, v}] = 2;
                        }

                        robot.move(directionMap[moveID]);
                    }
                }
                //之前没有目标 随便走
                else{
                    int moveID = -1;

                    for(int i = 0; i < 4; i++){
                        int u = robotX + moves[i].first;
                        int v = robotY + moves[i].second;

                        if(canMoveOn(u, v) && !m_lockMap.count({u, v})){
                            //判断方向锁
                            if(m_directionLockMap.count({robotX, robotY}) && m_directionLockMap[{robotX, robotY}] == i)
                                continue;

                            moveID = i;
                            break;
                        }
                    }

                    int u = robotX + moves[moveID].first;
                    int v = robotY + moves[moveID].second;

                    m_lockMap.insert({u, v});

                    //方向锁
                    if(moveID == 0){
                        m_directionLockMap[{u, v}] = 1;
                    }
                    else if(moveID == 1){
                        m_directionLockMap[{u, v}] = 0;
                    }
                    else if(moveID == 2){
                        m_directionLockMap[{u, v}] = 3;
                    }
                    else if(moveID == 3){
                        m_directionLockMap[{u, v}] = 2;
                    }

                    robot.move(directionMap[moveID]);
                }


            }
        }
        //送货机器人
        else{
            //指定去的berth
            int berthID = goods[robot.goodID].berthID;

            int moveID = -1, minDist = INF;
//            //能否留在原地
//            if(!m_lockMap.count({robotX, robotY})){
//                minDist = m_berthDepth[berthID][robotX][robotY];
//            }

            //是否需要移动
            for(int i = 0; i < 4; i++){
                int u = robotX + moves[i].first;
                int v = robotY + moves[i].second;

                if(canMoveOn(u, v) && !m_lockMap.count({u, v})){
                    //判断方向锁
                    if(m_directionLockMap.count({robotX, robotY}) && m_directionLockMap[{robotX, robotY}] == i)
                        continue;

                    if(m_berthDepth[berthID][u][v] < minDist){
                        minDist = m_berthDepth[berthID][u][v];
                        moveID = i;
                    }
                }
            }

            // 对机器人发出指令, 并锁定下一帧的位置
            if (moveID == -1) {
                //不移动 锁定当前位置
                m_lockMap.insert({robotX, robotY});
                cerr << "We cannot find a feasible next grid for robot" + to_string(robot.id) + " with goods!!!" << endl;
            }
            else {
                int u = robot.x + moves[moveID].first;
                int v = robot.y + moves[moveID].second;

                m_lockMap.insert({u, v});

                //方向锁
                if(moveID == 0){
                    m_directionLockMap[{u, v}] = 1;
                }
                else if(moveID == 1){
                    m_directionLockMap[{u, v}] = 0;
                }
                else if(moveID == 2){
                    m_directionLockMap[{u, v}] = 3;
                }
                else if(moveID == 3){
                    m_directionLockMap[{u, v}] = 2;
                }

                robot.move(directionMap[moveID]);

                if(m_map[u][v] == 'B'){
                    int berthID = m_pos2Berth[{u, v}];
                    if(berths[berthID].transportTime+frameID <= maxFrameID){
                        berths[berthID].goodsWorth.push(goods[robot.goodID].worth);
                        berths[berthID].sumOfGoodsWorth += goods[robot.goodID].worth;
                    }
                    m_scoreOfRobot2Berth += goods[robot.goodID].worth;
                    m_scoreOfBerth[berthID] += goods[robot.goodID].worth;

                    robot.pull();
                }
            }
        }
    }

//    for(int i = 0; i < numOfRobots; i++){
//        for(int j = i+1; j < numOfRobots; j++){
//            // 横向相遇
//            if(robots[j].x == robots[i].x && abs(robots[j].y-robots[i].y) == 1){
//                if(robots[j].orders.size() > 0 && robots[i].orders.size() > 0){
//                    if((robots[j].orders.back().back() == '2' && robots[i].orders.back().back() == '2')
//                       || (robots[j].orders.back().back() == '3'&& robots[i].orders.back().back() == '3')){
//                        robots[j].orders.pop_back();
//                    }
//                }
//            }
//            // 纵向相遇
//            if(robots[j].y == robots[i].y && abs(robots[j].x-robots[i].x) == 1){
//                if(robots[j].orders.size() > 0 && robots[i].orders.size() > 0){
//                    if((robots[j].orders.back().back() == '0' && robots[i].orders.back().back() == '0')
//                       || (robots[j].orders.back().back() == '1' && robots[i].orders.back().back() == '1')){
//                        robots[j].orders.pop_back();
//                    }
//                }
//            }
//        }
//    }


}

void Schedule::initIgnoreBerths(int frameID, const vector<Robot>& robots, const vector<Berth>& berths, const vector<Boat>& boats){

//    m_ignoreBerths.clear();
//
////    // 1. 基本约束: 把所有货物已经溢出的泊位禁止掉
////    for (int i = 0; i < numOfBerths; ++i) {
////        if (berths[i].loadingSpeed == 0) {
////            m_ignoreBerths.insert(i);
////            continue;
////        }
////        int loadingTime = (berths[i].goodsWorth.size() + berths[i].loadingSpeed - 1) / berths[i].loadingSpeed;
////        if (frameID + loadingTime + berths[i].transportTime + epsilonFrame >= maxFrameID) {
////            // 该泊位上的东西已经足够其工作到最后一个有效帧
////            m_ignoreBerths.insert(i);
////        }
////    }
//
//    // 全部往有船的港口运
//    if(frameID >= 13500){
////        m_ignoreBerths.insert(0);
////        m_ignoreBerths.insert(1);
////        m_ignoreBerths.insert(2);
////        m_ignoreBerths.insert(3);
////        m_ignoreBerths.insert(4);
//
//        for(int i = 0; i < numOfBerths; i++){
//            m_ignoreBerths.insert(i);
//        }
//        for(int i = 0; i < numOfBerths; i++){
//            for(int j = 0; j < numOfBoats; j++){
//                if(boats[j].berthID == i && boats[j].status == 1){
//                    m_ignoreBerths.erase(i);
//                }
//            }
//        }
//        cerr << m_ignoreBerths.size() << endl;
//
//        if(m_ignoreBerths.size() == 10)
//            m_ignoreBerths.clear();
//    }

    m_ignoreBerths.clear();

    // 1. 基本约束: 把所有货物已经溢出的泊位禁止掉
    for (int i = 0; i < numOfBerths; ++i) {
        if (berths[i].loadingSpeed == 0) {
            m_ignoreBerths.insert(i);
            continue;
        }
        // 卸货需要的时间
        int loadingTime = (berths[i].goodsWorth.size() + berths[i].loadingSpeed - 1) / berths[i].loadingSpeed;
        // 枚举其它轮船来此泊位需要的时间, 我们取其最小值作为基本约束
        int comingTime = maxFrameID;
        for (auto& boat : boats) {
            if (boat.berthID != i) {
                // 1. 轮船的 目标泊位 不是 本泊位
                if (boat.status == 0) {
                    // 在移动途中
                    if (boat.prevBerthID == -1) {
                        // 由虚拟点出发
                        comingTime = min(comingTime, berths[i].transportTime);
                    } else if (boat.prevBerthID == i) {
                        // 由本泊位出发
                        comingTime = 1;// 一帧指令即可回去载货
                        break;
                    } else {
                        // 由其它泊位出发
                        comingTime = min(comingTime, framesFromBerth2Berth);
                    }
                } else {
                    // 在虚拟点或者在泊位上
                    if (boat.berthID != -1) {
                        // 在其它泊位
                        comingTime = min(comingTime, framesFromBerth2Berth);
                    } else {
                        // 在虚拟点
                        comingTime = min(comingTime, berths[i].transportTime);
                    }
                }
            } else {
                // 2. 轮船的 目标泊位 是 本泊位
                if (boat.status == 1 || boat.status == 2) {
                    // 有轮船在泊位, 此值为 0
                    comingTime = 0;
                    break;
                } else {
                    // 检查走过来需要花多少时间
                    int t;
                    if (boat.prevBerthID != -1) {
                        // 从泊位过来
                        t = framesFromBerth2Berth - (frameID - boat.orderFrame);
                    } else {
                        // 从虚拟点过来
                        t = berths[boat.berthID].transportTime - (frameID - boat.orderFrame);
                    }
                    comingTime = min(comingTime, t);
                }
            }
        }
        if (frameID + loadingTime + comingTime + berths[i].transportTime + boatEpsilonFrame > maxFrameID) {
            // 该泊位上的东西已经足够其工作到最后一个有效帧
            m_ignoreBerths.insert(i);
        }
    }

//    if (frameID > m_frameIgnoreBerth && m_ignoreBerths.size() < m_ignoreNumOfBerth) {
//        // 未被禁止的泊位 ID
//        vector<int> berthIDs;
//        for (int i = 0; i < numOfBerths; ++i) {
//            if (m_ignoreBerths.count(i)) continue;// 已被禁止
//            berthIDs.push_back(i);
//        }
//
//        // 2. 根据之前的信息选择数据最优泊位留下来
//        // 把数据不好的泊位放在前面
//        sort(berthIDs.begin(), berthIDs.end(), [&](int bid1, int bid2) -> bool {
//            return m_scoreOfBerth[bid1] < m_scoreOfBerth[bid2];
//        });
//
//        // 如果一个泊位的数据不够好, 且该泊位不是该连通分量的唯一一个泊位, 就将该泊位禁止
//        for (int i = 0; i < berthIDs.size(); ++i) {
//            bool important = true;
//            int bid1 = berthIDs[i];
//            for (int j = i + 1; j < berthIDs.size(); ++j) {
//                int bid2 = berthIDs[j];
//                // 有性能更优秀的, 未溢出的泊位
//                if (m_partition[berths[bid2].x][berths[bid2].y] == m_partition[berths[bid1].x][berths[bid1].y]) {
//                    important = false;
//                    break;
//                }
//            }
//            if (!important) {
//                m_ignoreBerths.insert(bid1);
//                if (m_ignoreBerths.size() >= m_ignoreNumOfBerth) {
//                    // 已添加了足够的禁止泊位
//                    break;
//                }
//            }
//        }
//    }

    cerr << m_ignoreBerths.size() << endl;
//#ifdef __APPLE__
//    cerr << "[Schedule]: the num of ignore berths = " << m_ignoreBerths.size() << endl;
//#endif
}

void Schedule::summarize(int frameID,
                         vector<Robot>& robots,
                         vector<Good>& goods,
                         vector<Berth>& berths,
                         vector<Boat>& boats,
                         AStar* aStar,
                         int& goodsLeftPtr) {
    cerr << endl;
    cerr << "*********************\t Information of Score \t****************************" << endl;
//    cerr << "[Schedule]: summary !!!" << endl;
    cerr << "score of robot transport to berth: " << m_scoreOfRobot2Berth << endl;
    cerr << "score of boat transport to virtual point: " << m_scoreOfBoat2Virtual << endl;
    cerr << "\ttheir difference: " << m_scoreOfRobot2Berth - m_scoreOfBoat2Virtual << endl;

    // 打印轮船信息
    cerr << "*********************\t Information of Boats \t****************************" << endl;
    int sumBoatWorth = 0;
    cerr << "cavity of boat: " << boats[0].capacity << endl;
    for (auto& boat : boats) {
        cerr << "boat[" << boat.id << "]: statue = " << boat.status
        << ", \tberth = " << boat.berthID
        << ", \tlast order frame = " << boat.orderFrame
        << ", \tnum = " << boat.size
        << ", \tworth = " << boat.worth << endl;
        sumBoatWorth += boat.worth;
    }
    cerr << "\tsum of boat worth: " << sumBoatWorth << endl << endl;

    // 打印泊位信息
    cerr << "*********************\t Information of Berths \t****************************" << endl;
    int sumBerthWorth = 0;
    for (auto& berth : berths) {
        cerr << "berth[" << berth.id << "]: remain worth = " << berth.sumOfGoodsWorth
        << ", \tremain num = " << berth.goodsWorth.size() << endl;
        sumBerthWorth += berth.sumOfGoodsWorth;
    }
    cerr << "\tsum of berth worth: " << sumBerthWorth << endl << endl;
}

void Schedule::simulate(int frameID, vector<Robot> &robots, vector<Good> &goods, vector<Berth> &berths,
                        vector<Boat> &boats) {
    // TODO: implement
}

//更新部分
bool Schedule::canMoveOn(int u, int v) {
    if (u < 0 || u >= szOfMap || v < 0 || v >= szOfMap) return false;
    if (m_map[u][v] == '#' || m_map[u][v] == '*') return false;
    return true;
}

void Schedule::initDepthMap(int srcX, int srcY, int trgX, int trgY, vector<vector<int>>& depth) {

    queue<pair<int, int>> que;
    que.push({srcX, srcY});
    depth[srcX][srcY] = 0;
    if (srcX == trgX && srcY == trgY) return;

    while (!que.empty()) {
        auto [x, y] = que.front();
        que.pop();
        for (auto& [dx, dy] : moves) {
            int u = x + dx;
            int v = y + dy;
            if (canMoveOn(u, v) && depth[x][y] + 1 < depth[u][v]) {
                que.push({u, v});
                depth[u][v] = depth[x][y] + 1;
                if (u == trgX && v == trgY) return;
            }
        }

    }

}

void Schedule::initSquare(){
    m_pos2Square = vector<vector<int>> (szOfMap, vector<int> (szOfMap));
    int szOfSquare = szOfMap/m_squareLength;
    for(int i = 0; i < szOfMap; i++){
        for(int j = 0; j < szOfMap; j++){
            m_pos2Square[i][j] = (i/m_squareLength)*szOfSquare+(j/m_squareLength);
        }
    }
}

void Schedule::initSquareConnect(){
    m_pos2SquareConnect = vector<vector<int>> (szOfMap, vector<int> (szOfMap, -1));
    int szOfSquare = szOfMap/m_squareLength;
    int n_square = szOfSquare*szOfSquare;

    for(int sid = 0; sid < n_square; sid++){

        //计算区域左上角的坐标
        int xStart = (sid/szOfSquare)*m_squareLength;
        int yStart = (sid%szOfSquare)*m_squareLength;

        //用于记录该区域中存在多少个联通分量
        vector<vector<int>> squareFlagMap(m_squareLength, vector<int> (m_squareLength, INF));

        int count = 0;
        for(int i = 0; i < m_squareLength; i++){
            for(int j = 0; j < m_squareLength; j++){
                //未被标记 不是障碍物
                if(squareFlagMap[i][j] == INF && m_map[xStart+i][yStart+j] != '#' && m_map[xStart+i][yStart+j] != '*'){
                    queue<pair<int, int>> que;// [x, y]
                    squareFlagMap[i][j] = count;
                    que.push({i, j});

                    // 广搜扩散
                    while (!que.empty()) {
                        auto [iFront, jFront] = que.front();
                        que.pop();
                        for (auto& [di, dj] : moves) {
                            int iNow = iFront + di;
                            int jNow = jFront + dj;
                            //可达 未被标记 不是障碍物
                            if (0 <= iNow && iNow < m_squareLength && 0 <= jNow && jNow < m_squareLength && squareFlagMap[iNow][jNow] == INF && m_map[xStart+iNow][yStart+jNow] != '#' && m_map[xStart+iNow][yStart+jNow] != '*') {
                                squareFlagMap[iNow][jNow] = count;
                                que.push({iNow, jNow});
                            }
                        }
                    }
                    count++;
                }
                if(squareFlagMap[i][j] != INF)
                    m_pos2SquareConnect[xStart+i][yStart+j] = squareFlagMap[i][j];

            }
        }
    }
}

void Schedule::initSquareDepth(){
    //一行有多少个区域
    int szOfSquare = szOfMap/m_squareLength;
    int n_square = szOfSquare*szOfSquare;

    m_squareDepth = vector<vector<vector<vector<int>>>> (n_square);
    m_squareConnectCenter = vector<vector<pair<int, int>>> (n_square);

    for(int sid = 0; sid < n_square; sid++){

        //计算区域左上角的坐标
        int xStart = (sid/szOfSquare)*m_squareLength;
        int yStart = (sid%szOfSquare)*m_squareLength;

        //用于记录该区域中存在多少个分区
        map<int, pair<int, int>> squareConnectCenterMap;
        for(int i = 0; i < m_squareLength; i++){
            for(int j = 0; j < m_squareLength; j++){
                int connectID = m_pos2SquareConnect[xStart+i][yStart+j];
                if(connectID >= 0){
                    if(!squareConnectCenterMap.count(connectID)){
                        squareConnectCenterMap[connectID] = {i, j};
                    }
                    else{
                        auto [iLast, jLast] = squareConnectCenterMap[connectID];
                        if(abs(i-m_squareLength/2)+abs(j-m_squareLength/2)<abs(iLast-m_squareLength/2)+abs(jLast-m_squareLength/2)){
                            squareConnectCenterMap[connectID] = {i, j};
                        }
                    }
                }
            }
        }

        if(!squareConnectCenterMap.empty()){
            m_squareDepth[sid] = vector<vector<vector<int>>> (squareConnectCenterMap.size(), vector<vector<int>> (szOfMap, vector<int> (szOfMap, INF)));
            m_squareConnectCenter[sid] = vector<pair<int, int>> (squareConnectCenterMap.size());

            for(auto squareConnectCenter : squareConnectCenterMap){

                m_squareConnectCenter[sid][squareConnectCenter.first] = {xStart+squareConnectCenter.second.first, yStart+squareConnectCenter.second.second};

                //为square创建联通分量中心的深度图
                m_squareDepth[sid][squareConnectCenter.first] = vector<vector<int>> (szOfMap, vector<int> (szOfMap, INF));
                int xCenter = xStart+squareConnectCenter.second.first;
                int yCenter = yStart+squareConnectCenter.second.second;

                queue<pair<pair<int, int>, int>> que;// [x, y, depth]
                m_squareDepth[sid][squareConnectCenter.first][xCenter][yCenter] = 0;
                que.push({pair<int, int>(xCenter, yCenter), 0});

                // 广搜扩散
                while (!que.empty()) {
                    auto [pos, depth] = que.front();
                    que.pop();
                    auto [x, y] = pos;
                    for (auto& [dx, dy] : moves) {
                        int u = x + dx;
                        int v = y + dy;
                        if (canMoveOn(u, v) && m_squareDepth[sid][squareConnectCenter.first][u][v] > depth + 1) {
                            m_squareDepth[sid][squareConnectCenter.first][u][v] = depth + 1;
                            que.push({pair<int, int>(u, v), depth + 1});
                        }
                    }
                }

            }

        }
    }
}

void Schedule::updateLockMap(int frameID,
                             vector<Robot>& robots,
                             vector<Good>& goods,
                             vector<Berth>& berths,
                             vector<Boat>& boats,
                             AStar* aStar,
                             int& goodsLeftPtr) {

    // 更新 AStar 的锁定栅格数据
    aStar->clearLockMap();
    m_directionLockMap.clear();

    // 锁定恢复状态机器人 拿货机器人 卸货机器人
    for (auto& robot : robots) {
        // 被隔绝的机器人不处理
        if (m_partition[robot.x][robot.y] == -1) continue;
        // 恢复状态机器人锁定
        if (robot.moveStatus == 0){
            aStar->addLockedCoord(robot.x, robot.y);
        }
        else {
            if(robot.goodStatus == 0){
                // 检查有没有到达目标货物位置
                // 拿货机器人锁定
                if (robot.goodID != -1 && robot.goodID < goods.size() && goods[robot.goodID].isValid(frameID)) {
                    if (robot.x == goods[robot.goodID].x && robot.y == goods[robot.goodID].y) {
                        aStar->addLockedCoord(robot.x, robot.y);
                    }
                }
            }
            else{
                // 检查是否到达berth
                if (m_map[robot.x][robot.y] == 'B') {
                    // 卸货机器人锁定
                    aStar->addLockedCoord(robot.x, robot.y);
                }
            }
        }
    }

}

bool Schedule::existSufficientBoat(Berth berth, vector<Boat> &boats, int ignoreBoatID) {
    for (auto& boat : boats) {
        if (boat.id == ignoreBoatID || boat.berthID != berth.id) continue;
        // 三种情况:
        // a. 有其它轮船赶往该泊位
        // b. 有其它轮船在该泊位等待
        // c. 有其它轮船在服务, 且能装走所有的货物
        if (boat.status == 0 || boat.status == 2 || boat.capacity - boat.size > berth.goodsWorth.size()) {
            return true;
        }
    }
    return false;
}