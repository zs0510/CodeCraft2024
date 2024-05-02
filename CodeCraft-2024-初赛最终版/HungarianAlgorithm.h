//
// Created by zss on 2024/3/14.
//

#ifndef HUAWEI_HUNGARIANALGORITHM_H
#define HUAWEI_HUNGARIANALGORITHM_H

#if defined(__APPLE__) || defined(_WIN32)
#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <limits>
#else
#include <bits/stdc++.h>
#endif

using namespace std;

/*
 *  注意：
 *     1. 只支持 正 权重
 *     2. 目标为最大化总权重
 *
 *  用法:
 *     1. 计算三元组, 其格式为 tuple<double, int, int>, 分别为 权重(正), 机器人 id, 货物 id
 *     2. 获取匹配结果, 结果为 pair<int, int>, 分别为 机器人 id, 货物 id
 * */

class HungarianAlgorithm {
public:
    explicit HungarianAlgorithm(const vector<tuple<double, int, int>>& tuplesWeightRobotTarget) {
        init(tuplesWeightRobotTarget);
        execute();
    }

    vector<pair<int, int>> getMatchedRobotAndTarget();

    double getObjVal() {return m_sum;}

private:

    /****************************      成员函数      ****************************/

    void init(const vector<tuple<double, int, int>>& tuplesWeightRobotTarget);

    void execute();

    void addEdge(int x, int y, double w);
    bool check(int x);
    void bfs(int x);

    /****************************      成员变量      ****************************/

    // 机器人 / 货物 ID 与 下标的映射
    unordered_map<int, int> m_rid2idx, m_idx2rid, m_tid2idx, m_idx2tid;

    // 配对的结果
    unordered_map<int, int> m_matchedRobot2Good;
    unordered_map<int, int> m_matchedGood2Robot;

    int m_numRobots, m_numTargets;
    int m_maxNum;
    vector<int> m_matchRobots;
    vector<int> m_matchTargets;
    vector<int> m_pre;
    vector<bool> m_visitedRobots;
    vector<bool> m_visitedTargets;
    vector<double> m_lx;
    vector<double> m_ly;
    vector<vector<double>> m_weights;
    vector<double> m_slack;
    double m_INF;
    double m_sum = 0;
    queue<int> m_que;

};
#endif //HUAWEI_HUNGARIANALGORITHM_H
