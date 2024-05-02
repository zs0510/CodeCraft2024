#pragma GCC optimize(2)
#include <future>
#include <mutex>
#include <chrono>
#include <curl/curl.h>
#include "Schedule.h"

// Callback
size_t writeCallback(char *ptr, size_t size, size_t nmemb, std::string *data)
{
    data->append(ptr, size * nmemb);
    return size * nmemb;
}

struct Information {
    int rid, ans;
    string question;
    int sendFrame, responseFrame;
    Information(int _r, int _a, string _q, int _sF, int _rF): rid(_r), ans(_a), question(_q), sendFrame(_sF), responseFrame(_rF) {

    }
};

struct LLMData {
    // 本数据结构种所有 ID 均为私有 ID
    int sharedFrameID = 1;
    int sharedAskedCount = 0;
    int sharedAnsCount = 0;
    vector<int> sharedPrivateRobotAnswer = vector<int>(1000, -1);// 记录机器人提问问题的答案
    queue<pair<int, string>> sharedPrivateRobotQuestion;// 记录待向 LLM 发送问题的队列
    unordered_set<int> sharedAskedPrivateRobots;// 记录已经在提问队列中 / 在处理中的机器人 ID
    vector<Information> sharedInformation;
};

LLMData llmData;

// 互斥锁
std::mutex mtx;

// LLM Request
void sendPostRequest();
void summarizeLLM();

// 左上角为 [0, 0], 右下角为 [799, 799]
// '.': 空地
// '>': 陆地主干道, 机器人可在此重合
// '*': 海洋
// '~': 海洋主航道, 轮船可在此重合
// '#': 障碍
// 'R': 机器人购买地块, 同时也是陆地主干道
// 'S': 轮船购买地块, 同时也是海洋主航道
// 'B': 泊位
// 'K': 靠泊区
// 'C': 海陆立体交通地块, 机器人之间 / 轮船之间 均不能重合
// 'c': 海陆立体交通地块, 同时为主干道 / 主航道
// 'T': 交货点

/****************************      程序中的全局变量      ****************************/

GlobalData globalData;

// 注册辅助答题的副线程队列
const int numOfLLMProcessors = 3;
std::future<void> llmProcessors[numOfLLMProcessors] = {
        std::async(std::launch::async, sendPostRequest),
        std::async(std::launch::async, sendPostRequest),
        std::async(std::launch::async, sendPostRequest)
//        std::async(std::launch::async, sendPostRequest),
//        std::async(std::launch::async, sendPostRequest),
//        std::async(std::launch::async, sendPostRequest),
//        std::async(std::launch::async, sendPostRequest),
//        std::async(std::launch::async, sendPostRequest)
};

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

Schedule* schedule_xch = new Schedule;
MapFunction* mapFunction;
RobotSchedule* robotSchedule = new RobotSchedule;
BoatSchedule* boatSchedule = new BoatSchedule;

int main() {

    auto start = std::chrono::steady_clock::now();
    init();

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    cerr << "[Initialization]: cost " << (double)duration.count() / 1000 << "(ms)" << endl;

    double sumOfDuration = 0;
    int frameCount = 0;

    for(int frame = 1; frame <= maxFrameID; ++frame) {
        input();
        if (frame != globalData.frameID) {
            cerr << "[Error]: frame drop!!!" << endl;
        }
        frame = globalData.frameID;
        {
//            std::lock(mtx1, mtx2);
//            std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
//            std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
            std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
            ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData)
            sharedFrameID = frame;
        }

        auto start = std::chrono::steady_clock::now();
        process();
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//        cerr << "duration = " << (double)duration.count() / 1000 << "(ms)\n";
        sumOfDuration += (double)duration.count();
        ++frameCount;
        output();
    }
    cerr << "\n*********************\t Information of Efficiency \t****************************\n";
    cerr << "Total time consume = " << sumOfDuration / 1000 << "(ms)\n";
    cerr << "Total processed frames = " << frameCount << endl;
    cerr << "Average time consume = " << sumOfDuration / frameCount / 1000 << "(ms)\n";

    postprocess();

    return 0;
}

void init() {

    for (int i = 0; i < szOfMap; ++i) {
        scanf("%s", globalData.plat[i]);
        char okk[100];
        scanf("%s", okk);
        printf("OK\n");// 注意: 决赛地图输入要求读完每行都需要输出 OK 与之交互
        fflush(stdout);
    }

    scanf("%d", &globalData.numOfBerths);
    globalData.berths.resize(globalData.numOfBerths);
    for (int i = 0; i < globalData.numOfBerths; ++i) {
        scanf("%d", &globalData.berths[i].id);
        scanf("%d%d%d", &globalData.berths[i].x, &globalData.berths[i].y, &globalData.berths[i].loadingSpeed);
    }

    // 轮船容量
    scanf("%d%d", &globalData.boatCapacity0, &globalData.boatCapacity1);

    char okk[100];
    scanf("%s", okk);

    // 初始化后处理
    preprocess();

    // 后处理之后输出 OK
    printf("OK\n");
    fflush(stdout);
}

void preprocess() {

    cerr << "\n*********************\t Boats Capacity0 = " << globalData.boatCapacity0 << "\t****************************\n";
    cerr << "\n*********************\t Boats Capacity1 = " << globalData.boatCapacity1 << "\t****************************\n";
//    schedule = new Schedule(globalData);
    mapFunction = new MapFunction(globalData);
}

void input() {
    scanf("%d%d", &globalData.frameID, &globalData.money);

#if defined(_WIN32) || defined(__APPLE__)
    // 只在本地输出当前运行帧数与分数
    cerr << "[Schedule]: frame = " << globalData.frameID << ", money = " << globalData.money << endl;
#endif

    // 货物变化数量(包括过期, 被取走, 新增(新增<10))
    int num;
    scanf("%d", &num);
    for (int i = 0; i < num; ++i) {
        Good good;
        scanf("%d%d%d", &good.x, &good.y, &good.worth);
        if (good.worth != 0) {
            good.appearFrame = globalData.frameID;
            good.id = globalData.goods.size();
            if (good.worth > valuableGoodThreshold) {
                good.valuable = true;
            }
            globalData.goods.push_back(good);

        } else {
            // 货物过期 / 被取走
            mapFunction->removeGood(good.x, good.y);
        }
    }

    // 读入所有的机器人信息
    scanf("%d", &globalData.numOfRobots);
    for (int i = 0; i < globalData.numOfRobots; ++i) {
        if (i >= globalData.robots.size()) {
            globalData.robots.push_back(Robot());
        }
        auto& robot = globalData.robots[i];
        scanf("%d%d%d%d", &robot.id, &robot.goodsNum, &robot.x, &robot.y);
        if (robot.goodsNum == 2) {
            robot.type = 2;// 捕捉其他队伍的高级机器人信息
        }
    }

    // 读入所有的轮船信息
    scanf("%d", &globalData.numOfBoats);
    for (int i = 0; i < globalData.numOfBoats; ++i) {
        if (i >= globalData.boats.size()) {
            globalData.boats.push_back(Boat());
            globalData.boats.back().capacity = globalData.boatCapacity0;// 默认初始值
        }
        auto& boat = globalData.boats[i];
        scanf("%d%d%d%d%d%d\n", &boat.id, &boat.goodsNum, &boat.x, &boat.y, &boat.dir, &boat.status);
        if (boat.goodsNum > globalData.boatCapacity0) {
            boat.goodsNum = globalData.boatCapacity1;// 捕捉其它队伍的高级轮船
        }
    }

    // 读入本队伍控制的机器人信息
    scanf("%d", &globalData.numOfPrivateRobots);
    vector<int> private2Public;
    for (int i = 0; i < globalData.numOfPrivateRobots; ++i) {
        int rid;
        scanf("%d", &rid);
        if (!globalData.privateRobots.count(rid)) {
            globalData.privateRobots.insert(rid);
        }

        private2Public.push_back(rid);

//        if(i == 0){
//            globalData.robots[rid].type = 2;
//        }
    }

    // 读入处于答题状态的机器人的信息
    scanf("%d", &globalData.numOfAskedRobots);
    cerr << "globalData.numOfAskedRobots = " << globalData.numOfAskedRobots << endl;
    globalData.askedRobots.clear();
    int nextLineNum = -1;
    scanf("%d", &nextLineNum);

    vector<pair<int, string>> robotsAndQuestions;

    auto isNums = [](string& str) {
        for (auto& ch : str) {
            if (!isdigit(ch)) {
                return false;
            }
        }
        return true;
    };

    for (int i = 0; i < globalData.numOfAskedRobots; ++i) {

        int askedPrivateID = nextLineNum;
        string question;
        int ansCount = 0;
        while (true) {
            char tmpStr[200];
            scanf("%s", &tmpStr);
            string tmpNumStr = tmpStr;
            // 判断是不是 下一行的首数字
            if (isNums(tmpNumStr) && ansCount == 4) {
                nextLineNum = stoi(tmpNumStr);
                break;
            } else {
                for (int i = 1; i < tmpNumStr.length(); ++i) {
                    if (tmpNumStr[i - 1] >= 'A' && tmpNumStr[i - 1] <= 'D' && tmpNumStr[i] == '.') {
                        ++ansCount;
                    }
                }
                question += tmpStr;
                question += " ";
            }
        }

//        std::string line;
//        // 从标准输入读取一行文本，并存储到字符串变量 line 中
//        std::getline(std::cin, line);
//        while (line.empty()) {// 消除空格
//            std::getline(std::cin, line);
//        }
//        cerr << "line = " << line << endl;
//        int startIdx = 0;
//        while (startIdx < line.length()) {
//            if (isdigit(line[startIdx])) {
//                break;
//            }
//            ++startIdx;
//        }
//        int endIdx = startIdx + 1;
//        while (endIdx < line.length()) {
//            if (!isdigit(line[endIdx])) {
//                break;
//            }
//            ++endIdx;
//        }
//
//        int askedPrivateID = stoi(line.substr(startIdx, endIdx - startIdx));// [startIdx, endIdx)
//        string question = line.substr(endIdx, line.length());

        // 把私有 ID 转化为全局 ID, 这样更方便在机器人调度中处理



        int askedPublicID = private2Public[askedPrivateID];
        globalData.askedRobots.insert(askedPublicID);// 注意! 插入的机器人 ID 为全局 ID
        robotsAndQuestions.emplace_back(askedPrivateID, question);

//        {
//            // 上锁，确保只有一个线程可以访问共享资源
//            std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
//
//            // 问题已记录(在队列 / 在处理中) 或者 问题已解答待输出
//            if (llmData.sharedAskedPrivateRobots.count(askedPrivateID)
//                || llmData.sharedPrivateRobotAnswer[askedPrivateID] != -1) {
//                continue;
//            }
//
//            llmData.sharedPrivateRobotQuestion.push({askedPrivateID, question});
//            llmData.sharedAskedPrivateRobots.insert(askedPrivateID);
//
//            cerr << "askedPrivateID = " << askedPrivateID << endl;
//            cerr << "question = " << question << endl;
//            cerr << "robot[" << askedPrivateID << "] -> Q:" << question << "\n";
//        }

//        summarizeLLM();

    }

    // 读入本队伍控制的轮船信息
//    scanf("%d", &globalData.numOfPrivateBoats);
    globalData.numOfPrivateBoats = nextLineNum;
    cerr << "globalData.numOfPrivateBoats = " << globalData.numOfPrivateBoats << endl;
//    cerr << "bid = ";
    for (int i = 0; i < globalData.numOfPrivateBoats; ++i) {
        int bid;
        scanf("%d", &bid);
        if (!globalData.privateBoats.count(bid)) {
            globalData.privateBoats.insert(bid);
        }
//        cerr << bid << ",";
    }
//    cerr << "\n";

    char okk[100];
    scanf("%s", okk);
//    cerr << okk << "\n";

    for (auto [askedPrivateID, question] : robotsAndQuestions) {
        // 上锁，确保只有一个线程可以访问共享资源
//        std::lock(mtx1, mtx2);
//        std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
//        std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
        std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
        ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData)
        // 问题已记录(在队列 / 在处理中) 或者 问题已解答待输出
        if (sharedAskedPrivateRobots.count(askedPrivateID)
            || sharedPrivateRobotAnswer[askedPrivateID] != -1) {
            continue;
        }

        cerr << "askedPrivateID = " << askedPrivateID << endl;
        cerr << "question = " << question << endl;
//        cerr << "question.length() = " << question.length() << "\n";
//        question.resize(200);
        cerr << "robot[" << askedPrivateID << "] -> Q:" << question << "\n";

        sharedPrivateRobotQuestion.push({askedPrivateID, question});
        sharedAskedPrivateRobots.insert(askedPrivateID);
    }

//    if (!robotsAndQuestions.empty()) {
//        summarizeLLM();
//    }

}

void process() {

//    cerr << "[Schedule]: frame = " << globalData.frameID << endl;

//    // 更新各种信息
//    mapFunction->update(globalData);
//    // 调度
//    if (globalData.frameID <= 1000) {
//        // 只在前 1000 帧调度
//        schedule_xch->schedule(*robotSchedule, *boatSchedule, *mapFunction, globalData);
//    }

    // 检查副线程们是否已经完成计算任务
    for (int i = 0; i < numOfLLMProcessors; ++i) {
        if (llmProcessors[i].wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            // 如果副线程已经完成, 并有新的计算任务, 重新启动异步任务
            bool hasNewLLMRequest = false;
            {
                // 上锁，确保只有一个线程可以访问共享资源
//                std::lock(mtx1, mtx2);
//                std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
//                std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
                std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
                ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData)
                hasNewLLMRequest = !sharedPrivateRobotQuestion.empty();
            }
            if (hasNewLLMRequest) {
                cerr << "llmProcessors[i] = std::async(std::launch::async, sendPostRequest);" << endl;
                llmProcessors[i] = std::async(std::launch::async, sendPostRequest);
            }
        }
    }

    // 调度
    schedule_xch->schedule(*robotSchedule, *boatSchedule, *mapFunction, globalData);

//    if (globalData.frameID % 100 == 99) {
//        summarizeLLM();
//    }

//    if (globalData.frameID % 100 == 99)
//        mapFunction->summarize(globalData);
}

void output() {
    // 输出机器人指令
    auto rit = globalData.privateRobots.begin();
    for (int i = 0; i < globalData.numOfPrivateRobots; ++i, ++rit) {
        auto& robot = globalData.robots[*rit];

        // 检查是否是在回答问题的机器人
        {
            // 上锁，确保只有一个线程可以访问共享资源
//            std::lock(mtx1, mtx2);
//            std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
//            std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
            std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
            ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData)
            if (globalData.askedRobots.count(*rit) && sharedPrivateRobotAnswer[i] != -1) {
                robot.orders.clear();
                robot.ans(sharedPrivateRobotAnswer[i]);
                // 每个问题每个选手只有一次回答机会，回答成功后可顺利取货
                mapFunction->addAnsweredGood(mapFunction->getGoodID(robot.x, robot.y));
                sharedPrivateRobotAnswer[i] = -1;
            }
        }

        if (robot.orders.empty()) {// 无指令
            continue;
        }
        // 输出指令并清空指令
        for (auto& order : robot.orders) {
            switch (order.orderType) {
                case OrderType::RobotMove:
                    cout << "move " << i << " " << order.parameter << endl;
//                    cerr << "move " << i << " " << order.parameter << endl;
                    break;
                case OrderType::RobotGet:
                    cout << "get " << i << endl;
//                    cerr << "get " << i << endl;
                    break;
                case OrderType::RobotPull:
                    cout << "pull " << i << endl;
//                    cerr << "pull " << i << endl;
                    break;
                case OrderType::RobotAns:
                    cout << "ans " << i << " " << order.parameter << endl;
//                    cerr << "ans " << i << " " << order.parameter << endl;
                    {
                        // 上锁，确保只有一个线程可以访问共享资源
//                        std::lock(mtx1, mtx2);
//                        std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
//                        std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
                        std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
                        ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData)
                        ++sharedAnsCount;
                    }
                    break;
            }
        }
        robot.orders.clear();
        fflush(stdout);
    }

    // 输出轮船指令
    auto bit = globalData.privateBoats.begin();
    for (int i = 0; i < globalData.numOfPrivateBoats; ++i, ++bit) {
        auto& boat = globalData.boats[*bit];
        if (boat.order.empty()) {
            continue;
        }
        auto& order = boat.order.back();
        switch (order.orderType) {
            case OrderType::BoatShip:
                cout << "ship " << i << endl;
                break;
            case OrderType::BoatRot:
                cout << "rot " << i << " " << order.parameter << endl;
                break;
            case OrderType::BoatBerth:
                cout << "berth " << i << endl;
                break;
            case OrderType::BoatDept:
                cout << "dept " << i << endl;
                break;
            default:
                break;
        }
        boat.order.clear();
        fflush(stdout);
    }
    puts("OK");
    fflush(stdout);
}

void postprocess() {
//    mapFunction->summarize(globalData);
    summarizeLLM();
    schedule_xch->summarize(globalData);

}

void sendPostRequest()
{
    int sendFrame = -1, robotID = -1;
    string question;
    {
        // 上锁，确保只有一个线程可以访问共享资源
//        std::lock(mtx1, mtx2);
//        std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
//        std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
        std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
        ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData)
        if (sharedPrivateRobotQuestion.empty()) {// 加上条件 || sharedFrameID > 1000 后不闪退
            return;
        }
        sendFrame = sharedFrameID;
        robotID = sharedPrivateRobotQuestion.front().first;
        question = sharedPrivateRobotQuestion.front().second;
        sharedPrivateRobotQuestion.pop();
        cerr << "[LLM-Processor]: question = " << question << "\n";
        cerr << "[LLM-Processor]: send frame = " << sendFrame << "\n";
    }

//    auto start = std::chrono::steady_clock::now();

//    cerr << "input question: " << question << endl;
//    string prompt = "单选题：";
//    for (int i = 0; i < question.length(); ++i) {
//        auto& ch = question[i];
//        if ((i < 4 && isdigit(ch))) {
//            continue;
//        }
//        prompt += ch;
//    }
//    prompt += "。 只需要回答正确选项的字母：";
//    cerr << "processed prompt: " << prompt << endl;

    string prompt = "单选题：" + question + "。只需要回答正确选项的字母：";

    double temperature = 0.2;
    int top_k = 3;
    double top_p = 1.0;

    // 麦麦随心配队专属, 勿外传!!!
    const std::string API_URL = "https://infer-app-modelarts-cn-southwest-2.myhuaweicloud.com/v1/infers/0c968a33-6006-41b7-842a-528c21be854c";
    const std::string CONTENT_TYPE = "Content-Type: application/json;charset=utf-8";
    const std::string AUTH_TOKEN = "X-Apig-AppCode: 5b575781e2724992b2f232696dd57b970da4e4dd971745c3a5f70c3fd86c4780";

    CURL* curl = curl_easy_init();
    std::string response;

    // Set URL
    curl_easy_setopt(curl, CURLOPT_URL, API_URL.c_str());

    // Set request type: POST
    curl_easy_setopt(curl, CURLOPT_POST, 1L);

    // Set request params. Only 'prompt' included in this example, other parameters should be written by yourself.
    std::string postData = "{\"prompt\":\"" + prompt + "\", \"temperature\":" + std::to_string(temperature) + ", \"top_k\":" + std::to_string(top_k) + ", \"top_p\":" + std::to_string(top_p) + "}";


    cerr << postData << endl;

    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());

    // Set headers
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, CONTENT_TYPE.c_str());
    headers = curl_slist_append(headers, AUTH_TOKEN.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    // Set callback function and response.
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    // Send this request.
    CURLcode res = curl_easy_perform(curl);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    // TODO: 处理 LLM 返回的结果(response), A-0, B-1, C-2, D-4
    int ans = 0;
    if (res == CURLE_OK) {
        for (auto& ch : response) {
            if (ch >= 'A' && ch <= 'D') {
                ans = ch - 'A';
                break;
            }
        }
    }

    // Please process the response by yourself.
//    auto end = std::chrono::steady_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    {
        // 上锁，确保只有一个线程可以访问共享资源
//        std::lock(mtx1, mtx2);
//        std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
//        std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
        std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
        ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData)
        int responseFrame = sharedFrameID;
//        if (sharedFrameID > 1000) {// 测试代码: 放在此处不会报错
//            return;
//        }
        cerr << "response: " << response << endl;
        cerr << "[LLM-Processor]: response frame = " << responseFrame << "\n";
//        if (sharedFrameID > 1000 && robotID == 5) {// 测试代码
//            cerr << "line 649: robotID = " << robotID << endl;
////            sharedPrivateRobotAnswer[robotID] = ans;
////            sharedPrivateRobotAnswer[robotID % sharedPrivateRobotAnswer.size()] = ans;
//        } else {
//            sharedPrivateRobotAnswer[robotID] = ans;
//        }

//        if (sharedFrameID > 1000) {// 测试代码: 放在此处仍会报错
//            return;
//        }
        sharedPrivateRobotAnswer[robotID] = ans;
        // 注意: 不要在处理这个问答的时候就将该 robotID 从 sharedAskedPrivateRobots 移除(可能会造成重复提问的异常)
        sharedAskedPrivateRobots.erase(robotID);
        sharedInformation.push_back(Information(robotID, ans, question, sendFrame, responseFrame));
        ++sharedAskedCount;
    }
}

void summarizeLLM() {
    // 上锁，确保只有一个线程可以访问共享资源
//    std::lock(mtx1, mtx2);
//    std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
//    std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
    std::lock_guard<std::mutex> lock(mtx);// 锁的作用域与本语句作用域相等, 离开作用域时会自动解锁
    ORDERED_DECLARE(sharedFrameID, sharedPrivateRobotAnswer, sharedPrivateRobotQuestion, sharedAskedPrivateRobots, sharedInformation, sharedAskedCount, sharedAnsCount, llmData)
    if (sharedInformation.empty()) {
        return;
    }
    int sumFrames = 0, cntRequest = 0, maxDuration = -1, minDuration = maxFrameID;
    for (auto& node : sharedInformation) {
//        cerr << "send = " << send << ", response = " << response << endl;
        cerr << "\nrid = " << node.rid << ", \nq = " << node.question << ", \nans = " << node.ans
            << "\nsendFrame = " << node.sendFrame << ", \nresponseFrame = " << node.responseFrame << "\n";
        int cost = node.responseFrame - node.sendFrame;
        sumFrames += cost;
        ++cntRequest;
        maxDuration = max(maxDuration, cost);
        minDuration = min(minDuration, cost);
    }
    cerr << "\n*********************\t Information of LLM-Processor \t****************************\n";
    cerr << "Total frames = " << sumFrames << endl;
    cerr << "Requests count = " << cntRequest << endl;
    cerr << "Average frame = " << sumFrames / max(cntRequest, 1) << endl;
    cerr << "Max frame = " << maxDuration << endl;
    cerr << "Min frame = " << minDuration << endl;
    cerr << "llmData.sharedAskedCount = " << sharedAskedCount << endl;
    cerr << "llmData.sharedAnsCount = " << sharedAnsCount << endl;
    cerr << "llmData.sharedAskedPrivateRobots.size() = " << sharedAskedPrivateRobots.size() << endl;
    cerr << "llmData.sharedPrivateRobotQuestion.size() = " << sharedPrivateRobotQuestion.size() << endl;
}