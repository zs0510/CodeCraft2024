#include "DataStruct.h"

void Robot::get() {
//    string order = "get " + to_string(id);
//    cerr << order << endl;
    orders.push_back(Order(OrderType::RobotGet));
}

void Robot::pull() {
//    string order = "pull " + to_string(id);
//    cerr << order << endl;
    orders.push_back(Order(OrderType::RobotPull));
}

void Robot::move(int moveDirection) {
//    string order = "move " + to_string(id) + " " + to_string(int(moveDirection));
//    cerr << order << endl;
    orders.push_back(Order(OrderType::RobotMove, moveDirection));
}

void Robot::reset() {
    targetGoodID = -1;
}

void Robot::ans(int answer) {
//    string order = "ans " + to_string(id) + " " + to_string(answer);
//    cerr << order << endl;
    orders.push_back(Order(OrderType::RobotAns, answer));
}

bool Good::isValid(int frame) {
    // 贵重物品不会消失
    return frame < appearFrame + frameOfGoodExist || valuable;
}

void Boat::dept() {
    berthID = -1;
//    order = "dept " + to_string(id);
    order.clear();
    order.push_back(Order(OrderType::BoatDept));
}

void Boat::berth() {
//    order = "berth " + to_string(id);
    order.clear();
    order.push_back(Order(OrderType::BoatBerth));
}

void Boat::rot(RotationDirection rotationDirection) {
//    order = "rot " + to_string(id) + " " + to_string(int(rotationDirection));
    order.clear();
    order.push_back(Order(OrderType::BoatRot, int(rotationDirection)));
}

void Boat::ship() {
//    order = "ship " + to_string(id);
    order.clear();
    order.push_back(Order(OrderType::BoatShip));
}