#include "DataStruct.h"

bool Robot::get() {
    string order = "get " + to_string(id);
    orders.push_back(order);
    return true;
}

bool Robot::pull() {
    string order = "pull " + to_string(id);
    orders.push_back(order);
    return true;
}

bool Robot::move(int moveDirection) {
    string order = "move " + to_string(id) + " " + to_string(int(moveDirection));
    orders.push_back(order);
    return true;
}

bool Robot::reset() {
    path.clear();
    target.clear();
    goodID = -1;
    return true;
}

void Robot::updatePrevData() {
    prevX = x;
    prevY = y;
    prevGoodStatus = goodStatus;
    prevMoveStatus = moveStatus;
    prevGoodID = goodID;
}

bool Good::isValid(int frame) {
    return frame < appearFrame + frameOfGoodExist;
}

bool Boat::ship(int berthID, int frameID) {
    //TODO 检测
    orderFrame = frameID;
    order = "ship " + to_string(id) + " " + to_string(berthID);
    return true;
}

bool Boat::go(int frameID) {
    //TODO 检测
    orderFrame = frameID;
    order = "go " + to_string(id);
    return true;
}

void Boat::reset() {
    size = 0;
    worth = 0;
}

bool targetCompare(const Target t1, const Target t2){
    return (double)t1.worth / (t1.takeCost + t1.sendCost) > (double)t2.worth/(t2.takeCost+t2.sendCost);
}