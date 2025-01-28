#ifndef LOCKEDVALUE_H
#define LOCKEDVALUE_H

#include <shared_mutex>
#include <mutex>


template <class C>
class LockedValue
{
public:
    LockedValue()
        : m_value()
    {

    }

    C get() const
    {
        std::shared_lock lock(m_mutex);
        return m_value;
    }

    void set(const C& value)
    {
        std::unique_lock lock(m_mutex);
        m_value = value;
    }

private:
    C m_value;
    mutable std::shared_mutex m_mutex;
};

#endif // LOCKEDVALUE_H