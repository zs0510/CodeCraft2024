#pragma GCC optimize(2)
#include "Schedule.h"
#include <chrono>

// 左上角为 [0, 0], 右下角为 [199, 199]
// '.': 表示空地
// '*': 表示海洋
// '#': 表示障碍
// 'A': 表示机器人位置, 等价于 '.'
// 'B': 表示泊位位置, 大小为 4*4

bool berthCompare(const pair<int, int> p1, const pair<int, int> p2){
    return p1.second < p2.second;
}

// 初始化
void init();

// 预处理
void preprocess();

// 读入每帧命令
void input();

// 处理
void process();

// 输出每帧命令
void output();

// 后处理
void postprocess();

vector<Robot> robots(numOfRobots);
vector<Good> goods;
vector<Berth> berths(numOfBerths);
vector<Boat> boats(numOfBoats);

int frameID;
int score;
char plat[szOfMap+1][szOfMap+1];
int capacity;
int goodsLeftPtr = 0;

AStar* aStar = NULL;
Schedule* schedule;

int main() {
    auto start = std::chrono::steady_clock::now();
    init();
//    for(int j = 0; j < szOfMap; j++){
//        for(int k = 0; k < szOfMap; k++){
//            cerr << aStar->map[j][k];
//        }
//        cerr << endl;
//    }
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    cerr << "Time taken: " << (double)duration.count() / 1000 << " milliseconds" << endl;

    double sumOfDuration = 0;
    int frameCount = 0;

    for(int frame = 1; frame <= maxFrameID; ++frame) {
        input();
        if (frame != frameID) {
            cerr << "[Error]: frame drop!!!" << endl;
        }
        cerr << frameID << endl;
//        if(frameID >= 30){
//            output();
//            continue;
//        }

        frame = frameID;
        auto start = std::chrono::steady_clock::now();
        process();
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds >(end - start);
        sumOfDuration += (double)duration.count();
        ++frameCount;
        output();
//            cerr << "Time taken: " << (double)duration.count()/1000 << " milliseconds" << endl;

    }

    cerr << "sum = " << sumOfDuration << ", count = " << frameCount << endl;
    cerr << "avg = " << sumOfDuration / frameCount << endl;

    postprocess();

    return 0;
}

void init() {

    for (int i = 0; i < szOfMap; i ++)
        scanf("%s", plat[i]);


    //修改地图初始化
    for (int i = 0; i <= szOfMap; i++){
        plat[i][szOfMap] = '#';
        plat[szOfMap][i] = '#';
    }

    for (int i = 0; i < numOfBerths; i ++) {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berths[i].x, &berths[i].y, &berths[i].transportTime, &berths[i].loadingSpeed);
//        cerr << "berth id = " << i << ", t = " << berth[i].transportTime << ", v = " << berth[i].loadingSpeed << endl;
    }

    // 轮船容量, 轮船初始位置在虚拟点
    scanf("%d", &capacity);

    char okk[100];
    scanf("%s", okk);

    // 初始化后处理
    preprocess();

    // 后处理之后输出 OK
    printf("OK\n");
    fflush(stdout);
}

void preprocess() {

    // 初始化后处理
    for (int i = 0; i < numOfRobots; ++i){
        robots[i].id = i;
    }
    for (int i = 0; i < numOfBerths; ++i){
        berths[i].id = i;
    }
    for (int i = 0; i < numOfBoats; ++i){
        boats[i].id = i;
        boats[i].capacity = capacity;
    }

    // 按照读入的顺序对机器人坐标赋值
    int count = 0;
    for(int i = 0; i < szOfMap; i++) {
        for(int j = 0; j < szOfMap; j++) {
            if(plat[i][j] == 'A') {
                robots[count].x = i;
                robots[count].y = j;
                count++;
            }
        }
    }

    // 初始化AStar类
    aStar = new AStar(plat);

    // 初始化调度策略
    vector<vector<char>> scheduleMap(szOfMap, vector<char>(szOfMap));
    for (int i = 0; i < szOfMap; ++i) {
        for (int j = 0; j < szOfMap; ++j) {
            scheduleMap[i][j] = plat[i][j];
            if (scheduleMap[i][j] == 'A') {
                scheduleMap[i][j] = '.';
            }
        }
    }

    schedule = new Schedule(scheduleMap, berths, robots);

}

void input() {
    scanf("%d%d", &frameID, &score);

    // 新增货物的数量
    int num;
    scanf("%d", &num);
    for (int i = 0; i < num; ++i) {
        Good good;
        scanf("%d%d%d", &good.x, &good.y, &good.worth);
        good.appearFrame = frameID;
        good.id = goods.size();
        goods.push_back(good);
    }

    // 读入机器人信息
    for (int i = 0; i < numOfRobots; i ++) {
        robots[i].updatePrevData();
        scanf("%d%d%d%d", &robots[i].goodStatus, &robots[i].x, &robots[i].y, &robots[i].moveStatus);
    }

    // 读入轮船信息
    for (int i = 0; i < numOfBoats; i ++) {
        scanf("%d%d\n", &boats[i].status, &boats[i].berthID);
    }
    char okk[100];
    scanf("%s", okk);
}

void process() {
    schedule->optimize(frameID, robots, goods, berths, boats, aStar, goodsLeftPtr);
}

void output() {

    // 输出机器人指令
    for (int i = 0; i < numOfRobots; ++i) {
        if (robots[i].orders.empty()) {// 无指令
            continue;
        }
        // 输出指令并清空指令
        for(auto& order : robots[i].orders)
            cout << order << endl;
        robots[i].orders.clear();
    }

    // 输出轮船指令
    for (int i = 0; i < numOfBoats; ++i) {
        if (boats[i].order.empty()) {
            continue;
        }
        cout << boats[i].order << endl;
        boats[i].order.clear();
    }

    puts("OK");
    fflush(stdout);
}

void postprocess() {

//    cerr << "**********  PostProcess  **********" << endl;

//    int sumWorthOfBerths = 0, numGoodsOfBerths = 0;
//    for (auto& berth : berths) {
//        sumWorthOfBerths += berth.sumOfGoodsWorth;
//        numGoodsOfBerths += berth.goodsWorth.size();
//    }
//    cerr << "sum worth of berths: " << sumWorthOfBerths << endl;
//    cerr << "num goods of berths: " << numGoodsOfBerths << endl;
//
//    int numGoodsOfBoats = 0;
//    for (auto& boat : boats) {
//        numGoodsOfBoats += boat.size;
//    }
//    cerr << "num goods of boats: " << numGoodsOfBoats << endl;

    schedule->summarize(frameID, robots, goods, berths, boats, aStar, goodsLeftPtr);

//    cerr << "***********************************" << endl;
}