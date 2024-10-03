#include "fixed_size_map_queue.hpp"

template<class K, class V>
FixedSizeMapQueue<K, V>::FixedSizeMapQueue()
{
    std::cout << "FixedSizeMapQueue constructed" << std::endl;
    capacity_ = 20;
}

template<class K, class V>
FixedSizeMapQueue<K, V>::FixedSizeMapQueue(size_t capacity) : capacity_(capacity)
{
    std::cout << "FixedSizeMapQueue constructed" << std::endl;
}

template<class K, class V>
void FixedSizeMapQueue<K, V>::insert(const K& key, const V& value)
{
    if(map_.find(key) != map_.end())
    {
        map_[key] = value;
        return;
    }
    if(map_.size() >= capacity_)
    {
        K key_to_remove = order_.front();
        order_.pop();
        map_.erase(key_to_remove);
    }
    map_[key] = value;
    order_.push(key);
}

template<class K, class V>
V FixedSizeMapQueue<K, V>::get(const K& key)
{
    auto it = map_.find(key);
    if (it != map_.end()) {
        return it->second;
    } else {
        throw std::out_of_range("Key not found");
    }
}

template<class K, class V>
bool FixedSizeMapQueue<K, V>::contains(const K& key)
{
    return map_.find(key) != map_.end();
}

template<class K, class V>
size_t FixedSizeMapQueue<K, V>::size()
{
    return map_.size();
}

template<class K, class V>
std::pair<K, V> FixedSizeMapQueue<K, V>::begin()
{
    return std::make_pair(order_.front(), map_[order_.front()]);
}

template<class K, class V>
std::pair<K, V> FixedSizeMapQueue<K, V>::end()
{
    return std::make_pair(order_.back(), map_[order_.back()]);
}