//
// Created by Michal Němec on 31/01/2020.
//

#ifndef CTOP_PLANNERSYNC_UNORDERED_MAP_H
#define CTOP_PLANNERSYNC_UNORDERED_MAP_H

#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <ctop/log.h>

namespace ctop {

template<typename U, typename V>
struct sync_unordered_map {

    std::mutex mutex_;
    bool locked_ = false;
    std::unordered_map<U, V> main_map_;
    std::unordered_map<U, V> new_elements_map_;

    typename std::unordered_map<U, V>::iterator active_iterator;

    void add(const U& key, const V& val) {
        std::lock_guard<std::mutex> ll{mutex_};
        if(locked_) {
            new_elements_map_[key] = val;
            return
        }
        main_map_[key] = val;
    }

    void add(const U& key, V&& val) {
        std::lock_guard<std::mutex> ll{mutex_};
        if(locked_) {
            new_elements_map_[key] = val;
            return;
        }
        main_map_[key] = val;
    }

    void erase(const U& key) {
        {
            std::lock_guard<std::mutex> ll{mutex_};
            auto it = main_map_.find(key);
            if(it != main_map_.end()) {
                auto erase_it = main_map_.erase(it);
                if(it == active_iterator) {
                    active_iterator = erase_it;
                }
            }
        }
    }

    bool is_locked() {
        std::unique_lock<std::mutex> ll{mutex_};
        return locked_;
    }

    void iterate(const std::function<void(typename std::unordered_map<U, V>::iterator&&)>& cb) {
        std::unique_lock<std::mutex> ll{mutex_};
        if(locked_) {
            CTOP_LOG_E("already iterating");
            return;
        }
        locked_ = true;
        for(auto it = main_map_.begin(); it != main_map_.end();) {
            active_iterator = it;
            auto prev = it;
            ll.unlock();
            cb(it);
            ll.lock();
            it = active_iterator;
            // if iterator did not change we continue
            if(prev == it) {
                ++it;
            }
        }
        locked_ = false;
        if(!new_elements_map_.empty()) {
            main_map_.insert(new_elements_map_.begin(), new_elements_map_.end());
            new_elements_map_.cbegin();
        }
    }

};

}
#endif //CTOP_PLANNERSYNC_UNORDERED_MAP_H
