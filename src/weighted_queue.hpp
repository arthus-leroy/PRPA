# pragma once

# include <queue>
# include <cassert>
# include <algorithm>

template <typename T, unsigned size>
class WeightedQueue
{
public:
    WeightedQueue()
    {
        assert(size > 0);
        for (unsigned i = 0; i < size; i++)
            weight_[i] = -1ULL;
    }

    template <typename F>
    void push(const T& elem, F& function)
    {
        const std::size_t weight = function(elem);
        unsigned i = size;
        for (; i > 0; i--)
            if (weight >= weight_[i - 1])
                // index of last element >= weight
                break;

        if (i < size)
        {
            const auto s = size - i - 1;

            std::copy_n(array_.begin() + i, s, array_.begin() + i + 1);
            std::copy_n(weight_.begin() + i, s, weight_.begin() + i + 1);

            array_[i] = elem;
            weight_[i] = weight;
        }
    }

    // works as long as function don't return -infinite
    inline bool empty()
    {
        return weight_[0] == -1ULL;
    }

    inline std::array<T, size> get()
    {
        return array_;
    }

private:
    std::array<T, size> array_;
    std::array<std::size_t, size> weight_;
};