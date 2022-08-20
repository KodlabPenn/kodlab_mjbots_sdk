/**
 * @file type_traits.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Common type traits objects and functions.
 * @date 8/10/22
 * 
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 *
 */

#pragma once

#include <type_traits>

namespace kodlab::type_traits {

namespace internal {

/**
 * @brief Implementation of `is_base_of_template` that requires a template match
 * and returns `std::true_type`
 * @cite https://stackoverflow.com/a/34672753
 * @tparam Base base class template
 * @tparam Ts template parameters
 * @return std::true_type
 */
template<template<typename...> class Base, typename...Ts>
std::true_type is_base_of_template_impl(const Base<Ts...> *);

/**
 * @brief Implementation of `is_base_of_template` that returns `std::false_type`
 * @cite https://stackoverflow.com/a/34672753
 * @tparam Base base class template
 * @return std::false_type
 */
template<template<typename...> class Base>
std::false_type is_base_of_template_impl(...);

} // internal

/**
 * @brief Static test to determine if a class template is a base of a concrete
 * class
 * @detail Uses template deduction in `is_base_of_template_impl` implementations
 * to determine if `Base` is a base class template to `Derived`.
 * @warning This expression will not work in cases of private inheritance, and
 * may not work in cases of multiple inheritance.
 * @cite  https://stackoverflow.com/a/34672753
 * @tparam Base base class template
 * @tparam Derived concrete class based on a class template
 * @return `std::true_type` if `Base` is a base class template for `Derived`,
 * `std::false_type` otherwise
 */
template<template<typename...> class Base, typename Derived>
using is_base_of_template = decltype(internal::is_base_of_template_impl<Base>(
    std::declval<Derived *>()));

} // kodlab::type_traits
