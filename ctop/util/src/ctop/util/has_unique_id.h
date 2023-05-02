//
// Created by parallels on 5/13/19.
//

#ifndef CTOP_PLANNERHAS_UNIQUE_ID_H
#define CTOP_PLANNERHAS_UNIQUE_ID_H

#include <atomic>

/**
 * Initialize global uid counter per type
 * @tparam T
 * @tparam uid_t internal counter type
 */

template<typename T, typename uid_t = uint64_t>
struct has_unique_id
{
    using uid = uid_t;
    uid unique_id() const { return m_id_ ; }

    // do not copy or assign unique ids
    has_unique_id( const has_unique_id& ) : m_id_(++next_) {}
    has_unique_id( has_unique_id&& ) noexcept : m_id_(++next_) {}
    has_unique_id& operator= (const has_unique_id&) noexcept { return *this ; }

protected:
    has_unique_id() : m_id_(++next_) {}

private:
    uid m_id_ ;
    static std::atomic<uid> next_;

};

template<typename T, typename uid_t>
std::atomic<uid_t> has_unique_id<T, uid_t>::next_{0} ;

#endif //CTOP_PLANNERHAS_UNIQUE_ID_H
