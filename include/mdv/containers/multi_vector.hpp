#ifndef MDV_MULTI_VECTOR_CONTAINER_HPP
#define MDV_MULTI_VECTOR_CONTAINER_HPP

#include <cassert>
#include <cstddef>
#include <cstring>
#include <gsl/pointers>
#include <tuple>
#include <type_traits>

#include <mdv/macros.hpp>

namespace mdv {

/**
 * @brief vector-like container where each row is actually a tuple of elements
 *
 * @note All types must be copiable
 *
 */
template <typename... Ts>
struct multi_vector {
    using row    = std::tuple<Ts...>;  // NOLINT
    using vector = std::vector<row>;   // NOLINT

    template <typename... Args>
    multi_vector(Args... args) noexcept : _vec(std::forward<Args>(args)...) {}

    // clang-format off
    void push_back(const row& e)  { _vec.push_back(e); }
    void push_back(const row&& e) { _vec.push_back(e); }

    template <typename... Args>
    void emplace_back(Args... args) { _vec.emplace_back(std::forward<Args>(args)...); }

    void reserve(std::size_t n) { _vec.reserve(n); }

    MDV_NODISCARD row&       operator[](std::size_t i)       { return _vec[i]; }
    MDV_NODISCARD const row& operator[](std::size_t i) const { return _vec[i]; }
    MDV_NODISCARD row&       at(std::size_t i)               { return _vec.at(i); }
    MDV_NODISCARD const row& at(std::size_t i) const         { return _vec.at(i); }
    MDV_NODISCARD row&       front()                         { return _vec.front(); }
    MDV_NODISCARD const row& front() const                   { return _vec.front(); }
    MDV_NODISCARD row&       back()                          { return _vec.back(); }
    MDV_NODISCARD const row& back() const                    { return _vec.back(); }

    MDV_NODISCARD bool        empty() const    { return _vec.empty(); }
    MDV_NODISCARD std::size_t size() const     { return _vec.size(); }
    MDV_NODISCARD std::size_t capacity() const { return _vec.capacity(); }

    // clang-format on

protected:
    /**
     * struct subtype_iterator - Iterator for just one type of the stored entries.
     *
     */
    template <std::size_t Index, typename BaseIterator>
    struct subtype_iterator;

    /**
     * struct column_iterable - Contains begin()/end() information to iterate a given
     * column of the vector
     *
     */
    template <std::size_t Index, typename BaseIterator, typename BaseVector>
    struct column_iterable;

public:
    //  ___ _                 _
    // |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
    //  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
    //  | || ||  __/ | | (_| | || (_) | |  \__ \
    // |___|\__\___|_|  \__,_|\__\___/|_|  |___/
    //
    // clang-format off
    using row_iterator       = vector::iterator;       // NOLINT 
    using const_row_iterator = vector::const_iterator; // NOLINT 

    MDV_NODISCARD row_iterator       begin() noexcept       { return _vec.begin();  }
    MDV_NODISCARD row_iterator       end() noexcept         { return _vec.end();    }
    MDV_NODISCARD const_row_iterator begin() const noexcept { return _vec.begin();  }
    MDV_NODISCARD const_row_iterator end() const noexcept   { return _vec.end();    }
    MDV_NODISCARD const_row_iterator cbegin() noexcept      { return _vec.cbegin(); }
    MDV_NODISCARD const_row_iterator cend() noexcept        { return _vec.cend();   }

    template <std::size_t Index>
    MDV_NODISCARD auto column_iterator() { return column_iterable<Index, row_iterator, multi_vector>(this); }

    template <std::size_t Index>
    MDV_NODISCARD auto ccolumn_iterator() { return column_iterable<Index, const_row_iterator, const multi_vector>(this); }

    template <std::size_t Index>
    MDV_NODISCARD auto column_iterator() const { return column_iterable<Index, const_row_iterator, const multi_vector>(this); }

    // clang-format on

protected:
    template <std::size_t Index, typename BaseIterator>
    struct subtype_iterator : BaseIterator {
        static constexpr bool const_qualified = std::is_const_v<
                std::remove_reference_t<typename BaseIterator::reference>>;

        using e_type     = std::tuple_element_t<Index, row>;  // Element type
        using cq_element = std::conditional_t<const_qualified, const e_type, e_type>;
        using reference  = cq_element&;
        using pointer    = cq_element*;

        MDV_NODISCARD reference
        operator*() noexcept {
            return std::get<Index>(BaseIterator::operator*());
        }

        MDV_NODISCARD pointer
        operator->() noexcept {
            return &std::get<Index>(BaseIterator::operator*());
        }

    };  // struct column_element_iterator

    // clang-format off
    template <std::size_t Index, typename BaseIterator, typename BaseVector>
    struct column_iterable {
        using iterator = subtype_iterator<Index, BaseIterator>;

        iterator begin() { return iterator{_v->begin()}; }
        iterator end()   { return iterator{_v->end()}; }

        BaseVector* _v;
    };

    // clang-format on


private:
    vector _vec;
};


}  // namespace mdv

// #pragma GCC diagnostic pop

#undef MDV_MULTIVEC_DEBUG


#endif  // MDV_MULTI_VECTOR_CONTAINER_HPP
