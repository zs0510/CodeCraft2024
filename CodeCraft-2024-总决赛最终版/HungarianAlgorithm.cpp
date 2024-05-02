//
// Created by zss on 2024/3/14.
//

#include "HungarianAlgorithm.h"

vector<pair<int, int>> HungarianAlgorithm::getMatchedRobotAndTarget() {
    vector<pair<int, int>> ans;
    for (auto& [rid, gid] : m_matchedRobot2Good) {
        ans.emplace_back(rid, gid);
    }
    return ans;
}

void HungarianAlgorithm::init(const vector<tuple<double, int, int>>& tuplesWeightRobotTarget) {

    // 建立机器人与货物 到 内部下标 的映射
    for (auto& [w, rid, gid] : tuplesWeightRobotTarget) {
        if (!m_rid2idx.count(rid)) {
            m_rid2idx[rid] = m_rid2idx.size();
            m_idx2rid[m_idx2rid.size()] = rid;
        }
        if (!m_tid2idx.count(gid)) {
            m_tid2idx[gid] = m_tid2idx.size();
            m_idx2tid[m_idx2tid.size()] = gid;
        }
    }

    m_numRobots = m_idx2rid.size();
    m_numTargets = m_idx2tid.size();
    m_maxNum = max(m_numRobots, m_numTargets);
    m_INF = 1000000007.f;
    m_sum = 0;
    m_weights = vector<vector<double>>(m_maxNum, vector<double>(m_maxNum));
    m_matchRobots = vector<int>(m_maxNum, -1);
    m_matchTargets = vector<int>(m_maxNum, -1);
    m_pre = vector<int>(m_maxNum);
    m_visitedRobots = vector<bool>(m_maxNum);
    m_visitedTargets = vector<bool>(m_maxNum);
    m_lx = vector<double>(m_maxNum, -m_INF);
    m_ly = vector<double>(m_maxNum);
    m_slack = vector<double>(m_maxNum);

    // 添加边权
    for (auto& [w, rid, gid] : tuplesWeightRobotTarget) {
        addEdge(m_rid2idx[rid], m_tid2idx[gid], w);
    }

}

void HungarianAlgorithm::execute() {

    for (int i = 0; i < m_maxNum; ++i) {
        for (int j = 0; j < m_maxNum; ++j) {
            m_lx[i] = max(m_lx[i], m_weights[i][j]);
        }
    }

    for (int i = 0; i < m_maxNum; ++i) {
        fill(m_slack.begin(), m_slack.end(), m_INF);
        fill(m_visitedRobots.begin(), m_visitedRobots.end(), false);
        fill(m_visitedTargets.begin(), m_visitedTargets.end(), false);
        bfs(i);
    }

    for (int i = 0; i < m_maxNum; ++i) {
        if (m_weights[i][m_matchRobots[i]] > 0) {
            m_sum += m_weights[i][m_matchRobots[i]];
        } else {
            m_matchRobots[i] = -1;
        }
    }
    for (int i = 0; i < m_numRobots; ++i) {
//            cout << m_matchRobots[i] + 1 << " ";
        if (m_matchRobots[i] == -1) continue;
//        cout << m_idx2rid[i] << " -> " << m_idx2gid[m_matchRobots[i]] << endl;
        m_matchedRobot2Good[m_idx2rid[i]] = m_idx2tid[m_matchRobots[i]];
        m_matchedGood2Robot[m_idx2tid[m_matchRobots[i]]] = m_idx2rid[i];
    }
//    cout << "\m_maxNum";

//    if (m_numRobots != m_matchedRobot2Good.size()) {
//        cerr << "[HungarianAlgorithm]: 输入 " << m_numRobots << " 个机器人, "
//        << m_numTargets << " 个目标, 成功配对 "
//        << m_matchedRobot2Good.size() << " 个机器人." << endl;
//    }

//    cerr << "[HungarianAlgorithm]: 优化得到的总权重为 " << m_sum << endl;

}

void HungarianAlgorithm::addEdge(int x, int y, double weight) {
    m_weights[x][y] = fmax(weight, 0.f);
}

bool HungarianAlgorithm::check(int x) {
    m_visitedTargets[x] = true;
    if (m_matchTargets[x] != -1) {
        m_que.push(m_matchTargets[x]);
        m_visitedRobots[m_matchTargets[x]] = true;
        return false;
    }
    while (x != -1) {
        m_matchTargets[x] = m_pre[x];
        swap(x, m_matchRobots[m_pre[x]]);
    }
    return true;
}

void HungarianAlgorithm::bfs(int x) {
    while (!m_que.empty()) {
        m_que.pop();
    }
    m_que.push(x);
    m_visitedRobots[x] = true;
    while (true) {
        while (!m_que.empty()) {
            int u = m_que.front();
            m_que.pop();
            for (int v = 0; v < m_maxNum; ++v) {
                if (!m_visitedTargets[v]) {
                    double delta = m_lx[u] + m_ly[v] - m_weights[u][v];
                    if (m_slack[v] >= delta) {
                        m_pre[v] = u;
                        if (delta) {
                            m_slack[v] = delta;
                        } else if (check(v)) {
                            return;
                        }
                    }
                }
            }
        }
        double cost = m_INF;
        for (int i = 0; i < m_maxNum; ++i) {
            if (!m_visitedTargets[i]) {
                cost = min(cost, m_slack[i]);
            }
        }
        for (int i = 0; i < m_maxNum; ++i) {
            if (m_visitedRobots[i]) {
                m_lx[i] -= cost;
            }
            if (m_visitedTargets[i]) {
                m_ly[i] += cost;
            } else {
                m_slack[i] -= cost;
            }
        }
        for (int i = 0; i < m_maxNum; ++i) {
            if (!m_visitedTargets[i] && m_slack[i] == 0 && check(i)) {
                return;
            }
        }
    }
}