// Copyright 2026 Héctor Azpúrua
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
/// @brief `Span<T>`: a minimal C++17 stand-in for `std::span<T>` (dynamic extent only).

#ifndef VECTORFIELD_CORE__SPAN_HPP_
#define VECTORFIELD_CORE__SPAN_HPP_

#include <cassert>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace vectorfield_core
{

template <typename T>
class Span;

namespace detail
{

template <typename T>
struct IsSpan : std::false_type
{
};

template <typename T>
struct IsSpan<Span<T>> : std::true_type
{
};

// A container is span-compatible when it has data() and size() and its element type converts to
// T by a qualification conversion only (std::vector<X> -> Span<const X>, never X -> Base).
template <typename Container, typename T, typename = void>
struct IsSpanCompatible : std::false_type
{
};

template <typename Container, typename T>
struct IsSpanCompatible<
  Container, T,
  std::void_t<
    decltype(std::declval<Container &>().data()), decltype(std::declval<Container &>().size())>>
: std::is_convertible<
    std::remove_pointer_t<decltype(std::declval<Container &>().data())> (*)[], T (*)[]>
{
};

}  // namespace detail

/// @brief Non-owning view of a contiguous sequence: a pointer and a size.
///
/// Used for inputs that the core only reads during a call (polyline vertices, masks) and for
/// views into caller-owned buffers (`TrackerOutput::predicted`). A Span never owns, allocates or
/// frees memory; the viewed storage must outlive every use of the span. Construction from an
/// rvalue container is rejected at compile time, so a span can never be bound to a temporary
/// vector by accident.
/// @tparam T Element type; use `Span<const T>` for read-only views.
template <typename T>
class Span
{
public:
  /// Element type, possibly const.
  using element_type = T;
  /// Element type without cv-qualifiers.
  using value_type = std::remove_cv_t<T>;
  /// Size type.
  using size_type = std::size_t;
  /// Pointer type.
  using pointer = T *;
  /// Reference type.
  using reference = T &;
  /// Iterator type (a raw pointer).
  using iterator = T *;

  /// @brief An empty span.
  constexpr Span() noexcept = default;

  /// @brief Views `size` elements starting at `data` (data may be null only if size is 0).
  constexpr Span(T * data, size_type size) noexcept : data_(data), size_(size) {}

  /// @brief Views a built-in array.
  template <std::size_t N>
  constexpr Span(T (&array)[N]) noexcept  // NOLINT(google-explicit-constructor)
  : data_(array), size_(N)
  {
  }

  /// @brief Views an lvalue container with `data()` and `size()` (std::vector, std::array).
  template <
    typename Container, typename = std::enable_if_t<
                          !detail::IsSpan<std::remove_cv_t<Container>>::value &&
                          detail::IsSpanCompatible<Container, T>::value>>
  constexpr Span(Container & container) noexcept  // NOLINT(google-explicit-constructor)
  : data_(container.data()), size_(static_cast<size_type>(container.size()))
  {
  }

  /// @brief Converts `Span<U>` to `Span<T>` when only cv-qualification differs
  /// (`Span<X>` to `Span<const X>`).
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U (*)[], T (*)[]>>>
  constexpr Span(const Span<U> & other) noexcept  // NOLINT(google-explicit-constructor)
  : data_(other.data()), size_(other.size())
  {
  }

  /// @brief Pointer to the first element (may be null when empty).
  constexpr T * data() const noexcept { return data_; }
  /// @brief Number of elements.
  constexpr size_type size() const noexcept { return size_; }
  /// @brief True iff size() == 0.
  constexpr bool empty() const noexcept { return size_ == 0; }

  /// @brief Element i. Precondition: i < size().
  constexpr T & operator[](size_type i) const noexcept
  {
    assert(i < size_);
    return data_[i];
  }
  /// @brief First element. Precondition: !empty().
  constexpr T & front() const noexcept
  {
    assert(size_ > 0);
    return data_[0];
  }
  /// @brief Last element. Precondition: !empty().
  constexpr T & back() const noexcept
  {
    assert(size_ > 0);
    return data_[size_ - 1];
  }

  /// @brief Iterator to the first element.
  constexpr iterator begin() const noexcept { return data_; }
  /// @brief Iterator past the last element.
  constexpr iterator end() const noexcept { return data_ + size_; }

  /// @brief The first `count` elements. Precondition: count <= size().
  constexpr Span first(size_type count) const noexcept
  {
    assert(count <= size_);
    return Span(data_, count);
  }
  /// @brief The last `count` elements. Precondition: count <= size().
  constexpr Span last(size_type count) const noexcept
  {
    assert(count <= size_);
    return Span(data_ + (size_ - count), count);
  }
  /// @brief `count` elements starting at `offset`, clipped to the end of the span.
  /// Precondition: offset <= size().
  constexpr Span subspan(size_type offset, size_type count) const noexcept
  {
    assert(offset <= size_);
    const size_type available = size_ - offset;
    return Span(data_ + offset, count < available ? count : available);
  }

private:
  T * data_{nullptr};
  size_type size_{0};
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__SPAN_HPP_
