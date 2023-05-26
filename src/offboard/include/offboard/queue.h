#ifndef QUEUE_H_
#define QUEUE_H_

#include<ros/ros.h>

class ArrayQueue {
    private:
    int front, rear;
    public:
    int cap;
    //int *qArr;
    geometry_msgs::PoseStamped *qArr;
    ArrayQueue(int n) {cap = n; front = 0; rear = 0; qArr = new geometry_msgs::PoseStamped[cap];}
    void enQueue(geometry_msgs::PoseStamped x);
    geometry_msgs::PoseStamped deQueue();
    void printQueue();
    bool isFull();
    bool isEmpty();
    bool poseCompare(geometry_msgs::PoseStamped x, geometry_msgs::PoseStamped y);
    ~ArrayQueue();
};

#endif