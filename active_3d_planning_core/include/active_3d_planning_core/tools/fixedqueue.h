#include <deque>
#include <stdexcept>

template <typename T>
class FixedQueue {
private:
    std::deque<T> queue;
    size_t maxSize;

public:
    FixedQueue(size_t maxSize = 5) : maxSize(maxSize) {}

    void push(const T& value) {
        if (queue.size() == maxSize) {
            queue.pop_front();
        }
        queue.push_back(value);
    }

    void pop() {
        if (queue.empty()) {
            throw std::runtime_error("Queue is empty");
        }
        queue.pop_front();
    }

    T& front() {
        if (queue.empty()) {
            throw std::runtime_error("Queue is empty");
        }
        return queue.front();
    }

    const T& front() const {
        if (queue.empty()) {
            throw std::runtime_error("Queue is empty");
        }
        return queue.front();
    }

    size_t size() const {
        return queue.size();
    }

    bool empty() const {
        return queue.empty();
    }

    // Add iterator support
    typename std::deque<T>::iterator begin() {
        return queue.begin();
    }

    typename std::deque<T>::iterator end() {
        return queue.end();
    }

    typename std::deque<T>::const_iterator begin() const {
        return queue.begin();
    }

    typename std::deque<T>::const_iterator end() const {
        return queue.end();
    }
};
