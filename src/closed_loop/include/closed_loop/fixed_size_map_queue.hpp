#include <iostream>
#include <map>
#include <queue>

#include "opencv2/opencv.hpp"

template<class K, class V>
class FixedSizeMapQueue 
{
public:
    FixedSizeMapQueue();
    FixedSizeMapQueue(size_t capacity);
    void insert(const K& key, const V& value);
    V get(const K& key);
    bool contains(const K& key);
    size_t size();
    std::pair<K, V> begin();
    std::pair<K, V> end();

private:
    std::map<K, V> map_;
    std::queue<K> order_;
    size_t capacity_;
};