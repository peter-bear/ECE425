#include "positionQueue.h"
#include <Arduino.h>

// Constructor to initialize an empty queue
PositionQueue::PositionQueue() {
    front = nullptr;
    rear = nullptr;
}

// Check if the queue is empty
bool PositionQueue::isEmpty() {
    return front == nullptr;
}

// Enqueue operation to add (x, y) position
void PositionQueue::enqueue(int x, int y) {
    Node* newNode = new Node();
    newNode->data.x = x;
    newNode->data.y = y;
    newNode->next = nullptr;

    if (isEmpty()) {
        front = newNode;
        rear = newNode;
    } else {
        rear->next = newNode;
        rear = newNode;
    }

    // Serial.print("Enqueued: (");
    // Serial.print(x);
    // Serial.print(", ");
    // Serial.print(y);
    // Serial.println(")");
}

// Dequeue operation to remove and return (x, y) position
Position PositionQueue::dequeue() {
    Position empty = {-1, -1};  // Default value for empty queue
    if (isEmpty()) {
        Serial.println("Queue is empty!");
        return empty;
    }

    Node* temp = front;
    Position value = front->data;
    front = front->next;

    if (front == nullptr) {
        rear = nullptr;  // If the queue is empty, reset rear to null
    }

    delete temp;
    // Serial.print("Dequeued: (");
    // Serial.print(value.x);
    // Serial.print(", ");
    // Serial.print(value.y);
    // Serial.println(")");

    return value;
}
