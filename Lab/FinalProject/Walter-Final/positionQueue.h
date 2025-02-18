#ifndef POSITION_QUEUE_H
#define POSITION_QUEUE_H

// Structure to represent a position (x, y)
struct Position {
    int x;
    int y;
};

// Node structure for the linked list
struct Node {
    Position data;
    Node* next;
};

// Queue class for managing (x, y) positions
class PositionQueue {
private:
    Node* front;
    Node* rear;
    int size;

public:
    PositionQueue();  // Constructor
    bool isEmpty();  // Check if the queue is empty
    void enqueue(int x, int y);  // Add position to queue
    Position getByIndex(int index);
    Position dequeue();  // Remove position from queue
    int length() { return size; }  // Get the size of the queue
};

#endif  // POSITION_QUEUE_H
