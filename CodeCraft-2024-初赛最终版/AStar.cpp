#include "AStar.h"


AStar::AStar(char (*map)[szOfMap+1]){
    obstacle = vector<vector<bool>>(szOfMap+1, vector<bool>(szOfMap+1, false));

    for(int i = 0; i < szOfMap; i++){
        for(int j = 0; j < szOfMap; j++){
            if(map[i][j] == '#' || map[i][j] == '*')
                obstacle[i][j] = true;
        }
    }

    lockMap = vector<vector<bool>> (szOfMap+1, vector<bool> (szOfMap+1, false));
}

//TODO 调用前需要检查是否位于同一个分区
bool AStar::searchPath(int xStart, int yStart, int xEnd, int yEnd, vector<Coord>& path){

    Coord start(xStart, yStart);
    //递归结束的标志
    Coord flag(-1, -1, abs(xStart-xEnd)+abs(yStart-yEnd), 0);
    Coord end(xEnd, yEnd);
    Coord empty(-2, -2, 40000, 40000);

    vector<vector<Coord>> parentMap(szOfMap+1, vector<Coord>(szOfMap+1, empty));

    parentMap[start.x][start.y] = flag;

    vector<vector<int>> closeMap(szOfMap, vector<int>(szOfMap, 0));

    priority_queue<Coord, deque<Coord>, CoordCompare> openQueue;

    //构造开始点
    openQueue.push(start);

    vector<vector<int>> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

    while(!openQueue.empty()){
        //获得堆顶元素
        Coord top = openQueue.top();
        openQueue.pop();

        //访问过
        if(closeMap[top.x][top.y])
            continue;

        closeMap[top.x][top.y] = 1;

        //到达终点
        if(top == end){
            //构建路径
            path.push_back(end);
            auto p = parentMap[end.x][end.y];
            while(p != flag && p != end){
                path.push_back(p);
                p = parentMap[p.x][p.y];
            }
            return true;
        }

        Coord topPath = parentMap[top.x][top.y];

        for(auto& direction : directions){
            int xNew = top.x+direction[0];
            int yNew = top.y+direction[1];
            //坐标无效
            if(!isValidCoord(xNew, yNew))
                continue;

            //新节点在close列表里，忽略
            if(closeMap[xNew][yNew])
                continue;

            int distanceNew = abs(end.x-xNew)+ abs(end.y-yNew);

            openQueue.push(Coord{xNew, yNew, distanceNew, topPath.cost+1});

            //更新路径
            if(distanceNew+topPath.cost+1 < parentMap[xNew][yNew].cost+parentMap[xNew][yNew].distance){
                parentMap[xNew][yNew].x = top.x;
                parentMap[xNew][yNew].y = top.y;
                parentMap[xNew][yNew].cost = topPath.cost + 1;
                parentMap[xNew][yNew].distance = distanceNew;
            }
        }

    }
    return false;

}

// 每轮结束以后都需要清理lockMap！！！
void AStar::clearLockMap(){
//    for(int i = 0; i < szOfMap; i++){
//        for(int j = 0; j < szOfMap; j++){
//            lockMap[i][j] = false;
//        }
//    }
    for (auto& [x, y] : lockedCoords) {
        lockMap[x][y] = false;
    }
    lockedCoords.clear();
}

bool AStar::isLocked(int x, int y) {
    return lockMap[x][y];
}

bool AStar::isValidCoord(int x, int y){
    //在地图内
    if(x < 0 || x >= szOfMap || y < 0 || y >= szOfMap)
        return false;
    //非障碍物和海洋 且没有被lock
    return !obstacle[x][y] && !lockMap[x][y];
}

void AStar::addLockedCoord(int x, int y) {
    if (lockedCoords.count({x, y})) {
        return;
    }
    lockedCoords.insert({x, y});
    lockMap[x][y] = true;
}

void AStar::removeLockedCoord(int x, int y) {
    if (!lockedCoords.count({x, y})) {
        return;
    }
    lockedCoords.erase({x, y});
    lockMap[x][y] = false;
}