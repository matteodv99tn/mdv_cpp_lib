#ifndef MDV_MESH_ITERATOR_BASE_HPP
#define MDV_MESH_ITERATOR_BASE_HPP

#include <mdv/mesh/fwd.hpp>

namespace mdv::mesh {

template <typename Element, typename Index = long>
class MeshIterator {
public:
    // NOLINTBEGIN  naming conventions
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = Element;
    using pointer           = Element*;
    using reference         = Element&;

    // NOLINTEND

    MeshIterator<Element, Index>(const Mesh* mesh, const Index& index) noexcept : _v(mesh, index) {}

    Element&
    operator*() noexcept {
        return _v;
    }

    MeshIterator<Element, Index>&
    operator++() noexcept {
        ++_v._id;
        return *this;
    }

    MeshIterator<Element, Index>
    operator++(int) noexcept {
        MeshIterator<Element, Index> tmp = *this;
        ++_v._id;
        return tmp;
    }

    MeshIterator<Element, Index>&
    operator--() noexcept {
        --_v._id;
        return *this;
    }

    MeshIterator<Element, Index>
    operator--(int) noexcept {
        MeshIterator<Element, Index> tmp = *this;
        --_v._id;
        return tmp;
    }

    bool
    operator==(const MeshIterator<Element, Index>& other) const noexcept {
        return (_v._mesh == other._v._mesh) && (_v._id == other._v._id);
    }

    bool
    operator!=(const MeshIterator<Element, Index>& other) const noexcept {
        return !(*this == other);
    }

private:
    Element _v;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_ITERATOR_BASE_HPP
