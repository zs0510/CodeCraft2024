//
// Created by 许晨浩 on 2024/4/16.
//

#include "MapFunction.h"

MapFunction::MapFunction(GlobalData &globalData) {
    m_map = vector<vector<char>>(szOfMap, vector<char>(szOfMap));
    for (int i = 0; i < szOfMap; ++i) {
        for (int j = 0; j < szOfMap; ++j) {
            m_map[i][j] = globalData.plat[i][j];
        }
    }

    initDelivery(globalData);

    // 多线程初始化交货点块深度图
    thread th1(&MapFunction::initDeliveryDepth, this, ref(globalData), 0, m_deliveryBlock.size() / 2);
    thread th2(&MapFunction::initDeliveryDepth, this, ref(globalData), m_deliveryBlock.size() / 2, m_deliveryBlock.size());

    th1.join();
    th2.join();

    initBerth(globalData);

    // 多线程初始化泊位深度图
    thread th3(&MapFunction::initBerthDepth, this, ref(globalData), 0, m_berthBoatDepth.size() / 2);
    thread th4(&MapFunction::initBerthDepth, this, ref(globalData), m_berthBoatDepth.size() / 2, m_berthBoatDepth.size());

    th3.join();
    th4.join();

    initBerthWeight(globalData);

    initRobotCreator(globalData);
    initBoatCreator();

    initGoodLocalDepth();

    // 货物的起始下标为0
    goodLeftIndex = 0;
}

void MapFunction::initDelivery(GlobalData &globalData) {
    int n = m_map.size(), m = m_map[0].size();

    set<pair<int, int>> visitedDelivery;

    int count = 0;
    for(int i = 0; i < m; i++){
        for(int j = 0; j < n; j++){
            if(m_map[i][j] == 'T' && !visitedDelivery.count({i, j})){
                vector<pair<int, int>> deliveryBlock;

                deliveryBlock.push_back({i, j});
                visitedDelivery.insert({i, j});

                queue<pair<int, int>> que;

                m_pos2deliveryBlockID[{i, j}] = m_deliveryBlock.size();
                que.push({i, j});

                while (!que. empty()) {
                    auto [x, y] = que. front();
                    que.pop();
                    for (auto& [dx, dy] : moves) {
                        int u = x + dx;
                        int v = y + dy;
                        if (canMoveOn(u, v) && m_map[u][v] == 'T' && !visitedDelivery.count({u, v})){
                            m_pos2deliveryBlockID[{u, v}] = m_deliveryBlock.size();
                            que.push({u, v});
                            deliveryBlock.push_back({u, v});
                            visitedDelivery.insert({u, v});
                        }
                    }
                }
                m_deliveryBlock.push_back(deliveryBlock);
            }
        }
    }

    m_deliveryBlockGlobalDepth.resize(m_deliveryBlock.size());
}

void MapFunction::initDeliveryDepth(GlobalData& globalData, int left, int right){

    int n = m_map.size(), m = m_map[0].size();

    struct Node {
        int x, y, dir;
        Node(int _x, int _y, int _dir): x(_x), y(_y), dir(_dir) {

        }
    };

    for(int i = left; i < right; i++){
        auto& deliveryBlock = m_deliveryBlock[i];
        auto& globalDepth = m_deliveryBlockGlobalDepth[i];
        globalDepth = vector<vector<vector<int>>>(n, vector<vector<int>>(m, vector<int>(4, INF)));
        queue<Node> que;
        for(auto& delivery : deliveryBlock){
            for(int j = 0; j < 4; j++){
                que.push({delivery.first, delivery.second, j});
                // 得判断是不是合法坐标
                if (checkBoatCanMoveOn(delivery.first, delivery.second, j)) {
                    globalDepth[delivery.first][delivery.second][j] = 0;
                }
            }
        }

        while (!que.empty()) {
            auto node = que.front();
            que.pop();
            // 逆运算, 模拟三种操作
            int currentDepth = globalDepth[node.x][node.y][node.dir];
            vector<vector<int>> tmp;
            // 1. 假如是前进来此?
            tmp.push_back(getInvShip(node.x, node.y, node.dir));
            // 2. 假如是顺时针旋转来此?
            tmp.push_back(getInvClockwise(node.x, node.y, node.dir));
            // 3. 假如是逆时针旋转来此?
            tmp.push_back(getInvAnticlockwise(node.x, node.y, node.dir));
            for (auto& boat : tmp) {
                if (checkBoatCanMoveOn(boat[0], boat[1], boat[2])) {
                    int cost = isOnMainChannel(node.x, node.y, node.dir) ? 2 : 1;
                    if (currentDepth + cost < globalDepth[boat[0]][boat[1]][boat[2]] && currentDepth + cost <= maxBoat2DeliveryDistance) {
                        que.push(Node(boat[0], boat[1], boat[2]));
                        globalDepth[boat[0]][boat[1]][boat[2]] = currentDepth + cost;
                    }
                }
            }
        }
    }
}

void MapFunction::initBerth(GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    int n = m_map.size(), m = m_map[0].size();
    int numOfBerths = berths.size();

    // 1. 计算泊位区与靠泊区
    m_pos2Berth.clear();
    m_pos2NearBerth.clear();
    for (int berthID = 0; berthID < numOfBerths; ++berthID) {
        auto& berth = berths[berthID];
        set<pair<int, int>> visited;
        queue<pair<int, int>> que;
        que.push({berth.x, berth.y});
        visited.insert({berth.x, berth.y});
        while (!que.empty()) {
            auto [x, y] = que.front();
            que.pop();
            if (m_map[x][y] == 'K') {
                m_pos2NearBerth[{x, y}] = berthID;
            } else if (m_map[x][y] == 'B') {
                m_pos2Berth[{x, y}] = berthID;
            }
            for (auto& [dx, dy] : moves) {
                int u = x + dx, v = y + dy;
                if (!visited.count({u, v}) && canMoveOn(u, v) && (m_map[u][v] == 'K' || m_map[u][v] == 'B')) {
                    que.push({u, v});
                    visited.insert({u, v});
                }
            }
        }
    }

    m_berthRobotDepth.resize(numOfBerths);
    m_berthBoatDepth.resize(numOfBerths);

    // 4. 计算泊位方向
    for (int i = 0; i < numOfBerths; ++i) {
        // 检查方向, 右 或者 下
        auto& berth = berths[i];
        // 将泊位坐标移动到左上角, 存在多个泊位直接相连的情况?
        while (berth.x > 0 && globalData.plat[berth.x - 1][berth.y] == 'B') {
            --berth.x;
        }
        while (berth.y > 0 && globalData.plat[berth.x][berth.y - 1] == 'B') {
            --berth.y;
        }

        int x = berth.x, y = berth.y;
        bool isRight = true;
        if (berth.y + 2 >= m) {
            isRight = false;
        } else {
            if (m_map[x][y + 2] != 'B') {
                isRight = false;
            }
        }

        if (isRight) {
            berth.dir = 0;
        } else {
            berth.dir = 3;
        }
//        cerr << "[Schedule]: berth[" << i << "].dir = " << berth.dir << endl;
//
//        if (berth.dir == 0) {
//            cerr << m_map[x][y] << m_map[x][y + 1] << m_map[x][y + 2] << "\n";
//            cerr << m_map[x + 1][y] << m_map[x + 1][y + 1] << m_map[x + 1][y + 2] << "\n";
//        } else {
//            cerr << m_map[x][y] << m_map[x][y + 1] << "\n";
//            cerr << m_map[x + 1][y] << m_map[x + 1][y + 1] << "\n";
//            cerr << m_map[x + 2][y] << m_map[x + 2][y + 1] << "\n";
//        }
//        cerr << endl;
    }

    int numOfDelivery = m_deliveryBlock.size();
    m_distBerth2Delivery = vector<vector<int>>(numOfBerths, vector<int>(numOfDelivery, INF));
    m_minDistBerth2Delivery = vector<int>(numOfBerths, INF);
    for (int i = 0; i < numOfBerths; ++i) {
        auto& berth = berths[i];
        for (int j = 0; j < numOfDelivery; ++j) {
            if (berth.dir == 0) {
                // 向右, min -> max
                m_distBerth2Delivery[i][j] = max(m_deliveryBlockGlobalDepth[j][berth.x][berth.y][0], m_deliveryBlockGlobalDepth[j][berth.x + 1][berth.y + 2][1]);
            } else {
                // 向下
                m_distBerth2Delivery[i][j] = max(m_deliveryBlockGlobalDepth[j][berth.x][berth.y + 1][3], m_deliveryBlockGlobalDepth[j][berth.x + 2][berth.y][2]);
            }
            m_minDistBerth2Delivery[i] = min(m_minDistBerth2Delivery[i], m_distBerth2Delivery[i][j]);
        }
    }
}

void MapFunction::initBerthDepth(GlobalData& globalData, int left, int right){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    int n = m_map.size(), m = m_map[0].size();
    int numOfBerths = berths.size();

    auto initBerthNeighborDepth = [&](int berthID, vector<vector<int>>& depth, queue<pair<int, int>>& que) {
        auto& berth = berths[berthID];
        for (int dx = 0; berth.x + dx < n; ++dx) {
            bool existBerth = false;
            for (int dy = 0; berth.y + dy < m; ++dy) {
                int u = berth.x + dx, v = berth.y + dy;
                if (m_map[u][v] == 'B') {
                    que.push({u, v});
                    depth[u][v] = 0;
                    existBerth = true;
                } else {
                    break;
                }
            }
            if (!existBerth) break;
        }
    };

    // 2. 初始化泊位关于机器人的深度图
    for (int berthID = left; berthID < right; ++berthID) {
        auto& depth = m_berthRobotDepth[berthID];
        depth = vector<vector<int>>(n, vector<int>(m, INF));
        queue<pair<int, int>> que;// x, y
        initBerthNeighborDepth(berthID, depth, que);
        while (!que.empty()) {
            auto [x, y] = que.front();
            que.pop();
            for (auto& [dx, dy] : moves) {
                int u = x + dx;
                int v = y + dy;
                if (robotCanMoveOn(u, v) && depth[x][y] + 1 < depth[u][v]) {
                    que.push({u, v});
                    depth[u][v] = depth[x][y] + 1;
                }
            }
        }
    }

    struct Node {
        int x, y, dir;
        Node(int _x, int _y, int _dir): x(_x), y(_y), dir(_dir) {

        }
    };

    auto initBerthNeighborDirDepth = [&](int berthID, vector<vector<vector<int>>>& dirDepth, queue<Node>& que) {
        auto& berth = berths[berthID];
        for (int dx = 0; berth.x + dx < n; ++dx) {
            bool existBerth = false;
            for (int dy = 0; berth.y + dy < m; ++dy) {
                int u = berth.x + dx, v = berth.y + dy;
                if (m_map[u][v] == 'B') {
                    // 枚举方向
                    for (int dir = 0; dir < 4; ++dir) {
                        if (checkBoatCanMoveOn(u, v, dir)) {
                            que.push(Node(u, v, dir));
                            dirDepth[u][v][dir] = 0;
                        }
                    }
                    existBerth = true;
                } else {
                    break;
                }
            }
            if (!existBerth) break;
        }
    };

    // 3. 初始化泊位关于轮船的深度图
    for (int berthID = left; berthID < right; ++berthID) {
        auto& berth = berths[berthID];
        auto& dirDepth = m_berthBoatDepth[berthID];
        dirDepth = vector<vector<vector<int>>>(n, vector<vector<int>>(m, vector<int>(4, INF)));
        queue<Node> que;
        initBerthNeighborDirDepth(berthID, dirDepth, que);
        while (!que.empty()) {
            auto node = que.front();
            que.pop();
            // 逆运算, 模拟三种操作
            int currentDepth = dirDepth[node.x][node.y][node.dir];
            vector<vector<int>> tmp;
            // 1. 假如是前进来此?
            tmp.push_back(getInvShip(node.x, node.y, node.dir));
            // 2. 假如是顺时针旋转来此?
            tmp.push_back(getInvClockwise(node.x, node.y, node.dir));
            // 3. 假如是逆时针旋转来此?
            tmp.push_back(getInvAnticlockwise(node.x, node.y, node.dir));
            for (auto& boat : tmp) {
                if (checkBoatCanMoveOn(boat[0], boat[1], boat[2])) {
                    int cost = isOnMainChannel(node.x, node.y, node.dir) ? 2 : 1;
                    if (currentDepth + cost < dirDepth[boat[0]][boat[1]][boat[2]] && currentDepth + cost <= maxBoat2BerthDistance) {
                        que.push(Node(boat[0], boat[1], boat[2]));
                        dirDepth[boat[0]][boat[1]][boat[2]] = currentDepth + cost;
                    }
                }
            }
        }
    }
}

void MapFunction::initBerthWeight(GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    int n = m_map.size(), m = m_map[0].size();
    int numOfBerths = berths.size();

    vector<int> berthCount(numOfBerths, 0);

    for(int x = 0; x < n; x++){
        for(int y = 0; y < m; y++){
            // 机器人无法到达
            if(!robotCanMoveOn(x, y))
                continue;

            int minDist = INF;
            int berthID = -1;
            for(int i = 0; i < numOfBerths; i++){
                int dist = getBerthRobotDepth(i, x, y);
                if(dist < minDist){
                    minDist = dist;
                    berthID = i;
                }
            }

            if(berthID >= 0){
                ++berthCount[berthID];
            }
        }
    }

    int maxBerthCount = 0;
    for(int i = 0; i < numOfBerths; i++){
        maxBerthCount = max(maxBerthCount, berthCount[i]);
    }

    for(int i = 0; i < numOfBerths; i++){
        berths[i].weight = (double) berthCount[i] / maxBerthCount;
    }
}

void MapFunction::initRobotCreator(GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    //用bfs搜索该机器人购买地块的所有坐标
    set<pair<int, int>> visitedRobotCreator;

    int m = m_map.size(), n = m_map[0].size();
    for(int i = 0; i < m; i++){
        for(int j = 0; j < n; j++){
            if(m_map[i][j] == 'R' && !visitedRobotCreator.count({i, j})){
                vector<pair<int, int>> robotCreatorBlock;

                robotCreatorBlock.push_back({i, j});
                visitedRobotCreator.insert({i, j});

                queue<pair<int, int>> que;
                que.push({i, j});

                while (!que.empty()) {
                    auto [x, y] = que.front();
                    que.pop();
                    for (auto& [dx, dy] : moves) {
                        int u = x + dx;
                        int v = y + dy;
                        if (canMoveOn(u, v) && m_map[u][v] == 'R' && !visitedRobotCreator.count({u, v})) {
                            que.push({u, v});
                            robotCreatorBlock.push_back({u, v});
                            visitedRobotCreator.insert({u, v});
                        }
                    }
                }

                // 将离berth最近的robotCreator点放在最前面
                int minDistID = -1;
                int minDist = INF;
                for(int j = 0; j < robotCreatorBlock.size(); j++){
                    for(int k = 0; k < berths.size(); k++){
                        int dist = getBerthRobotDepth(k, robotCreatorBlock[j].first, robotCreatorBlock[j].second);

                        if (dist < minDist) {
                            minDist = dist;
                            minDistID = j;
                        }
                    }
                }

                if (minDistID > 0){
                    swap(robotCreatorBlock[0], robotCreatorBlock[minDistID]);
                }

                m_robotCreatorBlock.push_back(robotCreatorBlock);

//                m_robotCreatorBlock.push_back(robotCreatorBlock);
            }
        }
    }

}

void MapFunction::initBoatCreator(){
    int m = m_map.size(), n = m_map[0].size();

    for(int i = 0; i < m; i++){
        for(int j = 0; j < n; j++){
            if(m_map[i][j] == 'S'){
                m_boatCreator.push_back({i, j});
            }
        }
    }
}

void MapFunction::initGoodLocalDepth() {

    std::fill(begin(m_goodIsValid), end(m_goodIsValid), true);
    std::fill(begin(m_goodIsValuable), end(m_goodIsValuable), true);
    std::fill(&m_valuableGoodLocalDepth[0][0][0], &m_valuableGoodLocalDepth[0][0][0] + sizeof(m_valuableGoodLocalDepth) / sizeof(int), INF);
    std::fill(&m_cheapGoodLocalDepth[0][0][0], &m_cheapGoodLocalDepth[0][0][0] + sizeof(m_cheapGoodLocalDepth) / sizeof(int), INF);

    for (int i = 0; i < maxNumOfValuableGoodLocalDepth; ++i) {
        m_emptyValuableGoodLocalDepthID.push(i);
    }

    for (int i = 0; i < maxNumOfCheapGoodLocalDepth; ++i) {
        m_emptyCheapGoodLocalDepthID.push(i);
    }

}

void MapFunction::updateGoodLocalDepth(GlobalData& globalData) {

#if defined(_WIN32) || defined(__APPLE__)
//    cerr << "updateGoodLocalDepth begin...\n";
//    cerr << "***** valuable good local depth *****\n";
//    cerr << "m_emptyValuableGoodLocalDepthID.size() = " << m_emptyValuableGoodLocalDepthID.size() << endl;
//    cerr << "m_valuableGoodID2LocalDepthID.size() = " << m_valuableGoodID2LocalDepthID.size() << endl;
//    cerr << "\t sum is " << m_emptyValuableGoodLocalDepthID.size() + m_valuableGoodID2LocalDepthID.size() << endl;
//    cerr << "m_waitingValuableGoodBuildLocalDepth.size() = " << m_waitingValuableGoodBuildLocalDepth.size() << endl;
//    cerr << "***** cheap good local depth *****\n";
//    cerr << "m_emptyCheapGoodLocalDepthID.size() = " << m_emptyCheapGoodLocalDepthID.size() << endl;
//    cerr << "m_cheapGoodID2LocalDepthID.size() = " << m_cheapGoodID2LocalDepthID.size() << endl;
//    cerr << "\t sum is " << m_emptyCheapGoodLocalDepthID.size() + m_cheapGoodID2LocalDepthID.size() << endl;
//    cerr << "m_waitingCheapGoodBuildLocalDepth.size() = " << m_waitingCheapGoodBuildLocalDepth.size() << endl;
    if (m_emptyValuableGoodLocalDepthID.size() + m_valuableGoodID2LocalDepthID.size() != maxNumOfValuableGoodLocalDepth) {
        cerr << "内存管理错误! m_emptyValuableGoodLocalDepthID.size() + m_valuableGoodID2LocalDepthID.size() != maxNumOfValuableGoodLocalDepth\n";
    }
    if (m_emptyCheapGoodLocalDepthID.size() + m_cheapGoodID2LocalDepthID.size() != maxNumOfCheapGoodLocalDepth) {
        cerr << "内存管理错误! m_emptyCheapGoodLocalDepthID.size() + m_cheapGoodID2LocalDepthID.size() != maxNumOfCheapGoodLocalDepth\n";
    }
#endif
    int buildNumOfOneFrame = 12;
    vector<pair<int, int>> buildGoodDepth;
    // 先构造高价值货物的局部深度图
    while (!m_emptyValuableGoodLocalDepthID.empty() && !m_waitingValuableGoodBuildLocalDepth.empty() && buildGoodDepth.size() < buildNumOfOneFrame) {
        int gid = -1;
        while (!m_waitingValuableGoodBuildLocalDepth.empty()) {
            int tmpGid = m_waitingValuableGoodBuildLocalDepth.front();
            m_waitingValuableGoodBuildLocalDepth.pop();
            // 1. 有效货物, 建图; 2. 无效货物, 直接 pop
            if (m_goodIsValid[tmpGid]) {
                gid = tmpGid;
                break;
            }
        }
        if (gid == -1) {
            break;// 无有效资源
        }
        int did = m_emptyValuableGoodLocalDepthID.front();
        m_emptyValuableGoodLocalDepthID.pop();
        buildGoodDepth.push_back({gid, did});
    }

    // 再构造低价值货物的局部深度图
    while (!m_emptyCheapGoodLocalDepthID.empty() && !m_waitingCheapGoodBuildLocalDepth.empty() && buildGoodDepth.size() < buildNumOfOneFrame) {
        int gid = -1;
        while (!m_waitingCheapGoodBuildLocalDepth.empty()) {
            int tmpGid = m_waitingCheapGoodBuildLocalDepth.front();
            m_waitingCheapGoodBuildLocalDepth.pop();
            // 1. 有效货物, 建图; 2. 无效货物, 直接 pop
            if (m_goodIsValid[tmpGid] && globalData.goods[tmpGid].worth > minGoodWorth) {
                gid = tmpGid;
                break;
            }
        }
        if (gid == -1) {
            break;// 无有效资源
        }
        int did = m_emptyCheapGoodLocalDepthID.front();
        m_emptyCheapGoodLocalDepthID.pop();
        buildGoodDepth.push_back({gid, did});
    }

//    cerr << "buildGoodDepth.size() = " << buildGoodDepth.size() << endl;
    for (auto& [gid, did] : buildGoodDepth) {
//        cerr << "gid = " << gid << ", did = " << did << endl;
        if (m_goodIsValuable[gid]) {
            m_valuableGoodID2LocalDepthID[gid] = did;
//            cerr << "valuable: goodID(" << goodID << ") -> depthID(" << depthID << ")\n";
        } else {
            m_cheapGoodID2LocalDepthID[gid] = did;
        }

    }

    // 多线程初始化货物局部深度图
    thread th1(&MapFunction::calcGoodLocalDepth, this, ref(buildGoodDepth), 0, buildGoodDepth.size() / 2);
    thread th2(&MapFunction::calcGoodLocalDepth, this, ref(buildGoodDepth), buildGoodDepth.size() / 2, buildGoodDepth.size());

    th1.join();
    th2.join();

//    cerr << "updateGoodLocalDepth end...\n";

}

void MapFunction::calcGoodLocalDepth(const vector<pair<int, int>>& buildGoodDepth, int left, int right) {

    for (int i = left; i < right; ++i) {
        auto [goodID, depthID] = buildGoodDepth[i];
        calcSingleGoodLocalDepth(goodID, depthID);
    }

}

void MapFunction::calcSingleGoodLocalDepth(int goodID, int depthID) {
//    cerr << "calcSingleGoodLocalDepth: goodID = " << goodID << ", depthID = " << depthID << endl;
    int maxNumOfGoodLocalDepth = m_goodIsValuable[goodID] ? maxNumOfValuableGoodLocalDepth : maxNumOfCheapGoodLocalDepth;
    if (goodID < 0 || goodID >= m_goodLocation.size() || depthID < 0 || depthID >= maxNumOfGoodLocalDepth) {
        cerr << "goodID < 0 || goodID >= m_goodLocation.size() || depthID < 0 || depthID >= maxNumOfGoodLocalDepth\n";
        return;
    }

    auto& [srcX, srcY] = m_goodLocation[goodID];

    if (m_goodIsValuable[goodID]) {
        auto& depth = m_valuableGoodLocalDepth[depthID];
        for (int i = 0; i < sideOfValuableGoodLocalDepth; ++i) {
            for (int j = 0; j < sideOfValuableGoodLocalDepth; ++j) {
                depth[i][j] = INF;
            }
        }
        // 以 srcX, srcY 为中心构造深度图
        // 左上角为[0, 0], 右下角为[sideOfGoodLocalDepth-1, sideOfGoodLocalDepth-1]
        queue<pair<int, int>> que;// 存世界坐标
        que.push({srcX, srcY});
        auto [localSrcX, localSrcY] = getLocalCoord(goodID, srcX, srcY);
        depth[localSrcX][localSrcY] = 0;// 存局部坐标
        while (!que.empty()) {
            auto [worldX, worldY] = que.front();
            que.pop();
            auto [localX, localY] = getLocalCoord(goodID, worldX, worldY);
            for (auto& [dx, dy] : moves) {
                int worldU = worldX + dx;
                int worldV = worldY + dy;
                auto [localU, localV] = getLocalCoord(goodID, worldU, worldV);
                if (isValidLocalCoord(goodID, localU, localV) && robotCanMoveOn(worldX, worldY)) {
                    if (depth[localX][localY] + 1 < depth[localU][localV]) {
                        que.push({worldU, worldV});
                        depth[localU][localV] = depth[localX][localY] + 1;
                    }
                }
            }
        }



    } else {
        auto& depth = m_cheapGoodLocalDepth[depthID];
        for (int i = 0; i < sideOfCheapGoodLocalDepth; ++i) {
            for (int j = 0; j < sideOfCheapGoodLocalDepth; ++j) {
                depth[i][j] = INF;
            }
        }
        // 以 srcX, srcY 为中心构造深度图
        // 左上角为[0, 0], 右下角为[sideOfGoodLocalDepth-1, sideOfGoodLocalDepth-1]
        queue<pair<int, int>> que;// 存世界坐标
        que.push({srcX, srcY});
        auto [localSrcX, localSrcY] = getLocalCoord(goodID, srcX, srcY);
        depth[localSrcX][localSrcY] = 0;// 存局部坐标
        while (!que.empty()) {
            auto [worldX, worldY] = que.front();
            que.pop();
            auto [localX, localY] = getLocalCoord(goodID, worldX, worldY);
            for (auto& [dx, dy] : moves) {
                int worldU = worldX + dx;
                int worldV = worldY + dy;
                auto [localU, localV] = getLocalCoord(goodID, worldU, worldV);
                if (isValidLocalCoord(goodID, localU, localV) && robotCanMoveOn(worldX, worldY)) {
                    if (depth[localX][localY] + 1 < depth[localU][localV]) {
                        que.push({worldU, worldV});
                        depth[localU][localV] = depth[localX][localY] + 1;
                    }
                }
            }
        }

    }

}

bool MapFunction::isValidLocalCoord(int goodID, int localX, int localY) {
    if (m_goodIsValuable[goodID]) {
        if (localX < 0 || localX >= sideOfValuableGoodLocalDepth || localY < 0 || localY >= sideOfValuableGoodLocalDepth) {
            return false;
        }
    } else {
        if (localX < 0 || localX >= sideOfCheapGoodLocalDepth || localY < 0 || localY >= sideOfCheapGoodLocalDepth) {
            return false;
        }
    }

    return true;
}

// world.goodX <---> local.(sideOfGoodLocalDepth / 2 + 1)

pair<int, int> MapFunction::getWorldCoord(int goodID, int localX, int localY) {
    if (goodID < 0 || goodID >= m_goodLocation.size()) {
        return {-1, -1};
    }
    auto& [goodX, goodY] = m_goodLocation[goodID];
    int offset = m_goodIsValuable[goodID] ? (sideOfValuableGoodLocalDepth / 2 + 1) : (sideOfCheapGoodLocalDepth / 2 + 1);
    int worldX = localX - offset + goodX;
    int worldY = localY - offset + goodY;
    return pair<int, int>{worldX, worldY};
}

pair<int, int> MapFunction::getLocalCoord(int goodID, int worldX, int worldY) {
    if (goodID < 0 || goodID >= m_goodLocation.size()) {
        return {-1, -1};
    }
    auto& [goodX, goodY] = m_goodLocation[goodID];
    int offset = m_goodIsValuable[goodID] ? (sideOfValuableGoodLocalDepth / 2 + 1) : (sideOfCheapGoodLocalDepth / 2 + 1);
    int localX = worldX - goodX + offset;
    int localY = worldY - goodY + offset;
    return pair<int, int>{localX, localY};
}

void MapFunction::removeGoodLocalDepth(int goodID) {
    if (m_goodIsValuable[goodID]) {
        if (m_valuableGoodID2LocalDepthID.count(goodID)) {
            int depthID = m_valuableGoodID2LocalDepthID[goodID];
            m_valuableGoodID2LocalDepthID.erase(goodID);
            m_emptyValuableGoodLocalDepthID.push(depthID);// 释放 depthID
        }
//        cerr << "remove valuable good id...\n";
    } else {
        if (m_cheapGoodID2LocalDepthID.count(goodID)) {
            int depthID = m_cheapGoodID2LocalDepthID[goodID];
            m_cheapGoodID2LocalDepthID.erase(goodID);
            m_emptyCheapGoodLocalDepthID.push(depthID);// 释放 depthID
        }
//        cerr << "remove cheap good id...\n";
    }

}

int MapFunction::getGoodRobotDepth(int goodID, int x, int y){
//    // 测试: 直接使用曼哈顿距离
//    if (!m_pos2goodID.count({x, y})) {
//        return INF;
//    }
//    int goodX = m_goodLocation[id].first, goodY = m_goodLocation[id].second;
//    return abs(goodX - x) + abs(goodY - y);
    if (m_goodIsValuable[goodID]) {
        if (m_valuableGoodID2LocalDepthID.count(goodID)) {
            int depthID = m_valuableGoodID2LocalDepthID[goodID];
            if (depthID >= 0 && depthID < maxNumOfValuableGoodLocalDepth) {
                auto [localX, localY] = getLocalCoord(goodID, x, y);
                if (isValidLocalCoord(goodID, localX, localY)) {
                    return m_valuableGoodLocalDepth[depthID][localX][localY];
                }
            }
        }

    } else {
        if (m_cheapGoodID2LocalDepthID.count(goodID)) {
            int depthID = m_cheapGoodID2LocalDepthID[goodID];
            if (depthID >= 0 && depthID < maxNumOfCheapGoodLocalDepth) {
                auto [localX, localY] = getLocalCoord(goodID, x, y);
                if (isValidLocalCoord(goodID, localX, localY)) {
                    return m_cheapGoodLocalDepth[depthID][localX][localY];
                }
            }
        }
    }

    return INF;
}

int MapFunction::getBerthRobotDepth(int id, int x, int y){
    if (id != -1) {
        return m_berthRobotDepth[id][x][y];
    }
    int minDepth = INF;
    for (auto& depth : m_berthRobotDepth) {
        minDepth = min(minDepth, depth[x][y]);
    }
    return minDepth;
}

int MapFunction::getBerthBoatDepth(int id, int x, int y, int dir){
    return m_berthBoatDepth[id][x][y][dir];
}

int MapFunction::getDeliveryDepth(int x, int y, int dir, int deliveryID){
    if (deliveryID != -1) {
        return m_deliveryBlockGlobalDepth[deliveryID][x][y][dir];
    }
    int minDepth = INF;
    for (auto& depth : m_deliveryBlockGlobalDepth) {
        minDepth = min(minDepth, depth[x][y][dir]);
    }
    return minDepth;
}

int MapFunction::getNearBerthID(int x, int y){
    if(m_pos2NearBerth.count({x, y}))
        return m_pos2NearBerth[{x, y}];

    return -1;
}

int MapFunction::getBerthID(int x, int y){
    if(m_pos2Berth.count({x, y}))
        return m_pos2Berth[{x, y}];

    return -1;
}

int MapFunction::getMinDistBerth2Delivery(int id){
    return m_minDistBerth2Delivery[id];
}

const vector<vector<pair<int, int>>>& MapFunction::getRobotCreatorBlocks(){
    return m_robotCreatorBlock;
}

const vector<pair<int, int>>& MapFunction::getBoatCreators(){
    return m_boatCreator;
}

void MapFunction::update(GlobalData& globalData){
    DECLARE_VARIABLES(frameID, money, berths, robots, boats, goods, goodsLeftPtr, privateRobots, privateBoats, askedRobots, globalData)

    // 删除这一帧消失的货物
    for(auto& [x, y] : disappearedGoods){
        if (m_pos2goodID.count({x, y})) {
            int gid = m_pos2goodID[{x, y}];
            if (gid >= 0 && gid < maxGoodNum) {
                m_goodIsValid[gid] = false;
            }
            m_pos2goodID.erase({x, y});
            removeGoodLocalDepth(gid);
//            m_goodLocalDepth.erase(gid);
        }
    }

    disappearedGoods.clear();

    // 新增这一帧的货物
    for(int i = goodLeftIndex; i < goods.size(); i++){
        auto& good = goods[i];
        m_goodLocation.push_back({good.x, good.y});
        m_pos2goodID[{good.x, good.y}] = i;
#ifdef ONLY_PROCESS_VALUABLE_GOODS
        if (good.valuable) {
            m_waitingValuableGoodBuildLocalDepth.push(i);
            m_goodIsValuable[i] = true;
        } else {
            m_waitingCheapGoodBuildLocalDepth.push(i);
            m_goodIsValuable[i] = false;
        }
#else
        m_waitingGoodsBuildLocalDepth.push(i);
#endif
    }

    goodLeftIndex = goods.size();

    updateGoodLocalDepth(globalData);

}

void MapFunction::removeGood(int x, int y){
    disappearedGoods.insert({x, y});
}

int MapFunction::getGoodID(int x, int y) {
    if (!m_pos2goodID.count({x, y})) {
        return -1;
    }
    return m_pos2goodID[{x, y}];
}

bool MapFunction::isDisappearedGoods(int x, int y) {
    return disappearedGoods.count({x, y});
}

bool MapFunction::canMoveOn(int x, int y) {
    int n = m_map.size(), m = m_map[0].size();
    if (x < 0 || x >= n || y < 0 || y >= m) return false;
    return m_map[x][y] != '#';
}

bool MapFunction::robotCanMoveOn(int x, int y) {
    int n = m_map.size(), m = m_map[0].size();
    if (x < 0 || x >= n || y < 0 || y >= m) return false;
    if (m_map[x][y] == '#') return false;// 障碍
    if (m_map[x][y] == '*' || m_map[x][y] == '~' || m_map[x][y] == 'S') return false;// 海洋
    if (m_map[x][y] == 'K' || m_map[x][y] == 'T') return false;// 靠泊区, 交货点
    return true;
}

bool MapFunction::isSea(int x, int y) {
    int n = m_map.size(), m = m_map[0].size();
    if (x < 0 || x >= n || y < 0 || y >= m) return false;
    if (m_map[x][y] == '#') return false;// 障碍
    if (m_map[x][y] == '.' || m_map[x][y] == '>' || m_map[x][y] == 'R') return false;// 陆地
    return true;
}

bool MapFunction::isMainRoad(int x, int y) {
    int n = m_map.size(), m = m_map[0].size();
    if (x < 0 || x >= n || y < 0 || y >= m) return false;
    return m_map[x][y] == '>' || m_map[x][y] == 'R' || m_map[x][y] == 'c' || m_map[x][y] == 'B';
}

bool MapFunction::isMainChannel(int x, int y) {
    int n = m_map.size(), m = m_map[0].size();
    if (x < 0 || x >= n || y < 0 || y >= m) return false;
    return m_map[x][y] == '~' || m_map[x][y] == 'S' || m_map[x][y] == 'c' || m_map[x][y] == 'K' || m_map[x][y] == 'B' || m_map[x][y] == 'T';
}

bool MapFunction::isOnMainChannel(int x, int y, int dir) {
    auto boatBody = getBoatBody(x, y, dir);
    for (auto& [u, v] : boatBody) {
        if (isMainChannel(u, v)) {
            return true;
        }
    }
    return false;
}

bool MapFunction::checkBoatCanMoveOn(int x, int y, int dir) {
    auto body = getBoatBody(x, y, dir);
    // 检查是否可达
    for (auto& [u, v] : body) {
        if (!isSea(u, v)) //不是海洋
            return false;
    }

    return true;
}

int MapFunction::getBoatDir(int currentDir, RotationDirection rotation) {
    if (rotation == RotationDirection::Clockwise) {
        // 顺时针
        if (currentDir == 0) {
            return 3;
        } else if (currentDir == 3) {
            return 1;
        } else if (currentDir == 1) {
            return 2;
        } else if (currentDir == 2) {
            return 0;
        }
    } else {
        // 逆时针
        if (currentDir == 0) {
            return 2;
        } else if (currentDir == 3) {
            return 0;
        } else if (currentDir == 1) {
            return 3;
        } else if (currentDir == 2) {
            return 1;
        }
    }
    cerr << "[Schedule]: Error, you input a wrong dir!!!" << endl;
    return -1;
}

vector<pair<int, int>> MapFunction::getBoatBody(int x, int y, int dir) {
    vector<pair<int, int>> body;

    switch (dir) {
        case 0:
        {
            // 右
            body.push_back({x, y});body.push_back({x, y + 1});body.push_back({x, y + 2});
            body.push_back({x + 1, y});body.push_back({x + 1, y + 1});body.push_back({x + 1, y + 2});
        }
            break;
        case 1:
        {
            // 左
            body.push_back({x - 1, y - 2});body.push_back({x - 1, y - 1});body.push_back({x - 1, y});
            body.push_back({x, y - 2});body.push_back({x, y - 1});body.push_back({x, y});
        }
            break;
        case 2:
        {
            // 上
            body.push_back({x - 2, y});body.push_back({x - 2, y + 1});
            body.push_back({x - 1, y});body.push_back({x - 1, y + 1});
            body.push_back({x, y});body.push_back({x, y + 1});
        }
            break;
        case 3:
        {
            // 下
            body.push_back({x, y - 1});body.push_back({x, y});
            body.push_back({x + 1, y - 1});body.push_back({x + 1, y});
            body.push_back({x + 2, y - 1});body.push_back({x + 2, y});
        }
            break;
    }

    return body;
}

vector<int> MapFunction::getShip(int x, int y, int dir) {
    vector<int> ans(3);
    ans[0] = x + moves[dir].first;
    ans[1] = y + moves[dir].second;
    ans[2] = dir;
    return ans;
}

vector<int> MapFunction::getClockwise(int x, int y, int dir) {
    vector<int> ans(3);
    ans[0] = x + moves[dir].first * 2;
    ans[1] = y + moves[dir].second * 2;
    ans[2] = getBoatDir(dir, RotationDirection::Clockwise);
    return ans;
}

vector<int> MapFunction::getAnticlockwise(int x, int y, int dir) {
    vector<int> ans(3);
    switch (dir) {
        case 0:
            ans[0] = x + 1;
            ans[1] = y + 1;
            ans[2] = 2;
            break;
        case 1:
            ans[0] = x - 1;
            ans[1] = y - 1;
            ans[2] = 3;
            break;
        case 2:
            ans[0] = x - 1;
            ans[1] = y + 1;
            ans[2] = 1;
            break;
        case 3:
            ans[0] = x + 1;
            ans[1] = y - 1;
            ans[2] = 0;
            break;
    }
    return ans;
}

vector<int> MapFunction::getInvShip(int x, int y, int dir) {
    vector<int> ans(3);
    ans[0] = x - moves[dir].first;
    ans[1] = y - moves[dir].second;
    ans[2] = dir;
    return ans;
}

vector<int> MapFunction::getInvClockwise(int x, int y, int dir) {
    vector<int> ans(3);
    switch (dir) {
        case 0:// 现在右, 之前上
            ans[0] = x + 2;
            ans[1] = y;
            ans[2] = 2;
            break;
        case 1:// 现在左, 之前下
            ans[0] = x - 2;
            ans[1] = y;
            ans[2] = 3;
            break;
        case 2:// 现在上, 之前左
            ans[0] = x;
            ans[1] = y + 2;
            ans[2] = 1;
            break;
        case 3:// 现在下, 之前右
            ans[0] = x;
            ans[1] = y - 2;
            ans[2] = 0;
            break;
    }
    return ans;
}

vector<int> MapFunction::getInvAnticlockwise(int x, int y, int dir) {
    vector<int> ans(3);
    switch (dir) {
        case 0:// 现在右, 之前下
            ans[0] = x - 1;
            ans[1] = y + 1;
            ans[2] = 3;
            break;
        case 1:// 现在左, 之前上
            ans[0] = x + 1;
            ans[1] = y - 1;
            ans[2] = 2;
            break;
        case 2:// 现在上, 之前右
            ans[0] = x - 1;
            ans[1] = y - 1;
            ans[2] = 0;
            break;
        case 3:// 现在下, 之前左
            ans[0] = x + 1;
            ans[1] = y + 1;
            ans[2] = 1;
            break;
    }
    return ans;
}

void MapFunction::addAnsweredGood(int goodID) {
    m_answeredGoods.insert(goodID);
}

bool MapFunction::isAnsweredGood(int goodID) {
    return m_answeredGoods.count(goodID);
}