#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <set>
#include "ConstantDefinition.h"

using namespace std;

typedef struct Coord{
    int x, y;
    int cost, distance;

    Coord(int a, int b){
        x = a;
        y = b;
    }

    Coord(int a, int b, int c, int d){
        x = a;
        y = b;
        cost = c;
        distance = d;
    }

    bool operator == (Coord& other){
        return other.x == x && other.y == y;
    }

    bool operator != (Coord& other){
        return other.x != x || other.y != y;
    }
}Coord;

struct CoordCompare{
    bool operator()(const Coord& c1, const Coord& c2) const{
        // 这里可以根据节点的代价进行比较
        return c1.cost+c1.distance> c2.cost+c2.distance;
    }
};


class AStar {
public:
    AStar(char (*map)[szOfMap+1]);

    // 返回的路径是逆序的
    // 如果起点为 1, 终点为 5, 则返回的路径为 5-4-3-2-1
    bool searchPath(int xStart, int yStart, int xEnd, int yEnd, vector<Coord>& path);

    // 每轮结束以后都需要清理lockMap！！！
    void clearLockMap();

    void addLockedCoord(int x, int y);

    void removeLockedCoord(int x, int y);

    bool isLocked(int x, int y);

private:

    bool isValidCoord(int x, int y);

    vector<vector<bool>> obstacle;

    // 锁定地图上某些区域，防止机器人相撞
    set<pair<int, int>> lockedCoords;
    vector<vector<bool>> lockMap;
};
