#ifndef MDV_MACROS_HPP
#define MDV_MACROS_HPP

#define MDV_NODISCARD [[nodiscard]]


#if defined(__GNUC__) || defined(__clang__)
#define MDV_INLINE inline __attribute__((always_inline))
#elif defined(_MSC_VER)
#define MDV_INLINE __forceinline
#else
#define MDV_INLINE inline
#endif


#endif  // MDV_MACROS_HPP
