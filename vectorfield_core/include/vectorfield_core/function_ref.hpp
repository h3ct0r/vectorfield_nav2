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
/// @brief `FunctionRef<R(Args...)>`: a non-owning, never-allocating callable reference.

#ifndef VECTORFIELD_CORE__FUNCTION_REF_HPP_
#define VECTORFIELD_CORE__FUNCTION_REF_HPP_

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

namespace vectorfield_core
{

/// @brief Primary template; only the function-type specialisation is defined.
template <typename Signature>
class FunctionRef;

/// @brief Non-owning reference to a callable with signature `R(Args...)`.
///
/// Stores an object pointer and a function pointer; it never allocates and is trivially
/// copyable, which is why the hot path uses it instead of `std::function` (plan §3.2,
/// `FieldProbe`). Rules:
/// - A callable object (lambda, functor) is referenced, not copied: it must outlive every call.
///   Passing a temporary lambda directly as a function argument is safe for the duration of
///   that call.
/// - A function or function pointer is stored by value, so `FunctionRef f = &freeFunction;` is
///   safe.
/// - A default-constructed FunctionRef is empty; calling it is undefined behaviour. Test with
///   `explicit operator bool`.
/// - Calls are forwarded unchanged; a callable used on a `noexcept` path must not throw.
/// @tparam R Return type.
/// @tparam Args Argument types.
template <typename R, typename... Args>
class FunctionRef<R(Args...)>
{
public:
  /// @brief An empty reference.
  constexpr FunctionRef() noexcept = default;

  /// @brief References `callable` (or stores it, when it is a function or function pointer).
  template <
    typename F,
    typename = std::enable_if_t<
      !std::is_same_v<std::decay_t<F>, FunctionRef> && !std::is_member_pointer_v<std::decay_t<F>> &&
      std::is_invocable_r_v<R, F &, Args...>>>
  FunctionRef(F && callable) noexcept  // NOLINT(google-explicit-constructor)
  {
    using Decayed = std::decay_t<F>;
    if constexpr (
      std::is_pointer_v<Decayed> && std::is_function_v<std::remove_pointer_t<Decayed>>) {
      const Decayed function = callable;
      if (function != nullptr) {
        storage_.function = reinterpret_cast<void (*)()>(function);
        call_ = &callFunction<Decayed>;
      }
    } else {
      storage_.object = const_cast<void *>(static_cast<const void *>(std::addressof(callable)));
      call_ = &callObject<std::remove_reference_t<F>>;
    }
  }

  /// @brief True iff the reference is non-empty.
  explicit operator bool() const noexcept { return call_ != nullptr; }

  /// @brief Invokes the referenced callable. Precondition: non-empty.
  R operator()(Args... args) const { return call_(storage_, std::forward<Args>(args)...); }

private:
  union Storage {
    void * object;
    void (*function)();
  };
  using Call = R (*)(Storage, Args...);

  template <typename Object>
  static R callObject(Storage storage, Args... args)
  {
    if constexpr (std::is_void_v<R>) {
      std::invoke(*static_cast<Object *>(storage.object), std::forward<Args>(args)...);
    } else {
      return std::invoke(*static_cast<Object *>(storage.object), std::forward<Args>(args)...);
    }
  }

  template <typename Function>
  static R callFunction(Storage storage, Args... args)
  {
    const auto function = reinterpret_cast<Function>(storage.function);
    if constexpr (std::is_void_v<R>) {
      std::invoke(function, std::forward<Args>(args)...);
    } else {
      return std::invoke(function, std::forward<Args>(args)...);
    }
  }

  Storage storage_{nullptr};
  Call call_{nullptr};
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__FUNCTION_REF_HPP_
