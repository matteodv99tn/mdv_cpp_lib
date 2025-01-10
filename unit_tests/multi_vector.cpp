#include <stdexcept>

#include <catch2/catch_test_macros.hpp>
#include <mdv/containers/multi_vector.hpp>

using mdv::multi_vector;

using test_tuple  = std::tuple<double, int, char>;    // NOLINT naming
using test_vector = multi_vector<double, int, char>;  // NOLINT naming

namespace {

const std::size_t default_size = 10;

test_tuple
build_tuple() {
    return {4.2, 5, 'c'};
}

test_vector
build_vector(
        const std::size_t n_elems = default_size, const test_tuple& tpl = build_tuple()
) {
    return {n_elems, tpl};
}

test_vector
build_increasing_value(const std::size_t n_elems = default_size) {
    test_tuple  e{0.0, 0, 'a'};
    test_vector res;
    res.reserve(n_elems);

    for (std::size_t i = 0; i < n_elems; ++i) {
        res.push_back(e);
        ++std::get<int>(e);
        ++std::get<char>(e);
        std::get<double>(e) += 1;
    }

    return res;
}


}  // namespace

TEST_CASE("Multi vector", "[multi_vector][constructor][with size]") {
    test_vector vec(default_size);
    REQUIRE(vec.size() == default_size);
    REQUIRE(vec.front() == test_tuple());
    REQUIRE(vec.back() == test_tuple());
}

TEST_CASE("Multi vector", "[multi_vector][constructor][with size and value]") {
    const test_tuple elem = build_tuple();
    test_vector      vec(default_size, elem);
    REQUIRE(vec.size() == default_size);
    REQUIRE(vec.front() == elem);
    REQUIRE(vec.back() == elem);
}

TEST_CASE("Multi vector", "[multi_vector][copy constructor]") {
    const test_tuple elem = build_tuple();
    test_vector      vec(default_size, elem);
    REQUIRE(vec.size() == default_size);
    REQUIRE(vec.front() == elem);
    REQUIRE(vec.back() == elem);
}

TEST_CASE("Multi vector", "[multi_vector][reserve]") {
    test_vector vec;
    vec.reserve(default_size);
    REQUIRE(vec.size() == 0);
    REQUIRE(vec.capacity() >= default_size);
}

TEST_CASE("Multi vector", "[multi_vector][push_back][lvalue]") {
    test_tuple  e = build_tuple();
    test_vector vec;
    vec.push_back(e);
    REQUIRE(vec.size() == 1);
    REQUIRE(vec.front() == build_tuple());
}

TEST_CASE("Multi vector", "[multi_vector][push_back][rvalue]") {
    test_vector vec;
    vec.push_back(build_tuple());
    REQUIRE(vec.size() == 1);
    REQUIRE(vec.front() == build_tuple());
}

TEST_CASE("Multi vector", "[multi_vector][emplace_back]") {
    test_vector vec;
    vec.emplace_back(4.2, 5, 'c');
    REQUIRE(vec.size() == 1);
    REQUIRE(vec.front() == build_tuple());
}

TEST_CASE("Multi vector", "[multi_vector][access operator]") {
    const test_vector vec = build_increasing_value();
    for (auto i = 0; i < vec.size(); ++i)
        REQUIRE(static_cast<decltype(i)>(std::get<int>(vec[i])) == i);
}

TEST_CASE("Multi vector", "[multi_vector][at access operator]") {
    const test_vector vec = build_increasing_value();
    for (auto i = 0; i < vec.size(); ++i)
        REQUIRE(static_cast<decltype(i)>(std::get<int>(vec.at(i))) == i);
    REQUIRE_THROWS_AS(vec.at(vec.size()), std::out_of_range);
}

TEST_CASE("Multi vector", "[multi_vector][column iterator]") {
    test_vector vec = build_increasing_value(50);  // NOLINT magic numbers

    using double_iterator = decltype(vec.column_iterator<0>());
    static_assert(std::is_same_v<double_iterator::iterator::reference, double&>);
    static_assert(std::is_same_v<double_iterator::iterator::pointer, double*>);

    using cdouble_iterator = decltype(vec.ccolumn_iterator<0>());
    static_assert(std::is_same_v<cdouble_iterator::iterator::reference, const double&>);
    static_assert(std::is_same_v<cdouble_iterator::iterator::pointer, const double*>);

    double double_v = 0.0;
    int    int_v    = 0;
    char   char_v   = 'a';
    for (const auto& it : vec.column_iterator<1>()) REQUIRE(it == double_v++);
    for (const auto& it : vec.column_iterator<1>()) REQUIRE(it == int_v++);
    for (const auto& it : vec.column_iterator<2>()) REQUIRE(it == char_v++);
}

TEST_CASE("Multi vector", "[multi_vector][const column iterator]") {
    const test_vector vec = build_increasing_value(50);  // NOLINT magic numbers
    using double_iterator = decltype(vec.column_iterator<0>());
    static_assert(std::is_same_v<double_iterator::iterator::reference, const double&>);
    static_assert(std::is_same_v<double_iterator::iterator::pointer, const double*>);

    double double_v = 0.0;
    int    int_v    = 0;
    char   char_v   = 'a';
    for (const auto& it : vec.column_iterator<1>()) REQUIRE(it == double_v++);
    for (const auto& it : vec.column_iterator<1>()) REQUIRE(it == int_v++);
    for (const auto& it : vec.column_iterator<2>()) REQUIRE(it == char_v++);
}
