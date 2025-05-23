#ifndef PCD_PROCESS_HPP
#define PCD_PROCESS_HPP

#include <functional>
#include <grid_map_core/grid_map_core.hpp>

inline bool isFinite(const tf2::Quaternion& q) {
    return std::isfinite(q.x()) &&
           std::isfinite(q.y()) &&
           std::isfinite(q.z()) &&
           std::isfinite(q.w());
};

namespace std {

    template <>
    struct hash<grid_map::Index> {
        std::size_t operator()(const grid_map::Index& index) const {
            // Get hash of first and second index, and combine with xor, with left shift so padding exists
            return std::hash<int>()(index(0)) ^ (std::hash<int>()(index(1)) << 1);
        }
    };
};

struct equal_to_grid_map_index {
    bool operator()(const grid_map::Index& lhs, const grid_map::Index& rhs) const noexcept {
        return lhs(0) == rhs(0) && lhs(1) == rhs(1);
    }
};

#endif
