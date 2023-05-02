//
// Created by michalks on 28.07.19.
//

#ifndef CTOP_PLANNERQUEUEBUFFER_H
#define CTOP_PLANNERQUEUEBUFFER_H

#include <deque>
#include <condition_variable>

namespace ctop {

template<typename T, typename Alloc = std::allocator<T>, template<typename U = T, typename A = Alloc> class V = std::deque>
class QueueBuffer {

public:
    void add(const T& num) {
        std::unique_lock<std::mutex> locker(mu_);
        cond_.wait(locker, [this](){return buffer_.size() < size_;});
        buffer_.push_back(num);
        locker.unlock();
        cond_.notify_one();
    }

    void clear() {
        std::unique_lock<std::mutex> locker(mu_);
        buffer_.clear();
        locker.unlock();
        cond_.notify_one();
    }

    void clear_add(const T& val) {
        std::unique_lock<std::mutex> locker(mu_);
        buffer_.clear();
        buffer_.push_back(val);
        locker.unlock();
        cond_.notify_one();
    }

    bool empty() {
        std::unique_lock<std::mutex> locker(mu_);
        return buffer_.empty();
    }

    T remove() {
        std::unique_lock<std::mutex> locker(mu_);
        cond_.wait(locker, [this](){return buffer_.size() > 0;});
        T back = buffer_.front();
        buffer_.pop_front();
        locker.unlock();
        cond_.notify_one();
        return back;
    }

    std::vector<T> remove_all() {
        std::unique_lock<std::mutex> locker(mu_);
        cond_.wait(locker, [this](){return buffer_.size() > 0;});
        std::vector<T> out;
        out.reserve(buffer_.size());
        while(!buffer_.empty()) {
            T back = buffer_.front();
            buffer_.pop_front();
            out.push_back(back);
        }
        locker.unlock();
        cond_.notify_one();
        return out;
    }

    std::vector<T> state() {
        std::unique_lock<std::mutex> locker(mu_);
        std::vector<T> copy;
        for(const auto& it : buffer_) {
            copy.push_back(it);
        }
        return copy;
    }

    QueueBuffer() = default;

private:
    // Add them as member variables here
    std::mutex mu_;
    std::condition_variable cond_;

    // Your normal variables here
    V<T, Alloc> buffer_;
    const unsigned long size_ = std::numeric_limits<unsigned long>::max();

};

}
#endif //CTOP_PLANNERQUEUEBUFFER_H
