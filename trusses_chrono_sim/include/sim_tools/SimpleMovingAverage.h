#include <iostream>
#include <vector>
#include <queue>
#include <numeric>

class SimpleMovingAverage {
private:
    int windowSize;
    std::queue<double> window;
    double sum;
    double average;
    

public:
    // Constructor
    SimpleMovingAverage(int size) : windowSize(size), sum(0.0) {}
    
    // Add new value and compute the moving average
    double add(double value) {
        // Add new value to the window and update the sum
        window.push(value);
        sum += value;

        // If the window size exceeds the defined limit, remove the oldest value
        if (window.size() > windowSize) {
            sum -= window.front();
            window.pop();
        }

        average = sum / window.size(); // Calculate the current average

        return average;
    }

    // Get the current average
    double getAverage() const {
        return average;
    }
};
