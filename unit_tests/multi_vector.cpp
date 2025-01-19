#include <gtest/gtest.h>
#include <stdexcept>

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

TEST(MultiVectorTest, ConstructorWithSize) {
    test_vector vec(default_size);
    EXPECT_EQ(vec.size(), default_size);
    EXPECT_EQ(vec.front(), test_tuple());
    EXPECT_EQ(vec.back(), test_tuple());
}

TEST(MultiVectorTest, ConstructorWithSizeAndValue) {
    const test_tuple elem = build_tuple();
    test_vector      vec(default_size, elem);
    EXPECT_EQ(vec.size(), default_size);
    EXPECT_EQ(vec.front(), elem);
    EXPECT_EQ(vec.back(), elem);
}

TEST(MultiVectorTest, CopyConstructor) {
    const test_tuple elem = build_tuple();
    test_vector      vec(default_size, elem);
    EXPECT_EQ(vec.size(), default_size);
    EXPECT_EQ(vec.front(), elem);
    EXPECT_EQ(vec.back(), elem);
}

TEST(MultiVectorTest, Reserve) {
    test_vector vec;
    vec.reserve(default_size);
    EXPECT_EQ(vec.size(), 0);
    EXPECT_GE(vec.capacity(), default_size);
}

TEST(MultiVectorTest, PushBackLvalue) {
    test_tuple  e = build_tuple();
    test_vector vec;
    vec.push_back(e);
    EXPECT_EQ(vec.size(), 1);
    EXPECT_EQ(vec.front(), build_tuple());
}

TEST(MultiVectorTest, PushBackRvalue) {
    test_vector vec;
    vec.push_back(build_tuple());
    EXPECT_EQ(vec.size(), 1);
    EXPECT_EQ(vec.front(), build_tuple());
}

TEST(MultiVectorTest, EmplaceBack) {
    test_vector vec;
    vec.emplace_back(4.2, 5, 'c');
    EXPECT_EQ(vec.size(), 1);
    EXPECT_EQ(vec.front(), build_tuple());
}

TEST(MultiVectorTest, AccessOperator) {
    const test_vector vec = build_increasing_value();
    for (auto i = 0; i < vec.size(); ++i)
        EXPECT_EQ(static_cast<decltype(i)>(std::get<int>(vec[i])), i);
}

TEST(MultiVectorTest, AtAccessOperator) {
    const test_vector vec = build_increasing_value();
    for (auto i = 0; i < vec.size(); ++i) EXPECT_EQ(std::get<int>(vec.at(i)), i);
    ASSERT_THROW(vec.at(vec.size()), std::out_of_range);
}

TEST(MultiVectorTest, ColumnIterator) {
    const test_vector vec      = build_increasing_value(50);  // NOLINT magic numbers
    double            double_v = 0.0;
    int               int_v    = 0;
    char              char_v   = 'a';
    for (const auto& it : vec.column_iterator<0>()) ASSERT_EQ(it, double_v++);
    for (const auto& it : vec.column_iterator<1>()) ASSERT_EQ(it, int_v++);
    for (const auto& it : vec.column_iterator<2>()) ASSERT_EQ(it, char_v++);
}

TEST(MultiVectorTest, ConstColumnIterator) {
    const test_vector vec      = build_increasing_value(50);  // NOLINT magic numbers
    double            double_v = 0.0;
    int               int_v    = 0;
    char              char_v   = 'a';
    for (const auto& it : vec.column_iterator<0>()) ASSERT_EQ(it, double_v++);
    for (const auto& it : vec.column_iterator<1>()) ASSERT_EQ(it, int_v++);
    for (const auto& it : vec.column_iterator<2>()) ASSERT_EQ(it, char_v++);
}
