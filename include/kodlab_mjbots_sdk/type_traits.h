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
 * and returns std::true_type
 * @cite https://stackoverflow.com/a/34672753
 * @tparam C concrete class based on a class template
 * @tparam Ts base class template
 * @return std::true_type
 */
template<template<typename...> class C, typename...Ts>
std::true_type is_base_of_template_impl(const C<Ts...> *);

/**
 * @brief Implementation of `is_base_of_template` that returns std::false_type
 * @cite https://stackoverflow.com/a/34672753
 * @tparam C concrete class based on a class template
 * @return std::false_type
 */
template<template<typename...> class C>
std::false_type is_base_of_template_impl(...);

} // internal

/**
 * @brief Static test to determine if class template `Ts` is a base of class `C`
 * @warning This expression will not work in cases of private inheritance, and
 * may not work in cases of multiple inheritance.
 * @cite  https://stackoverflow.com/a/34672753
 * @tparam C concrete class based on a class template
 * @tparam Ts base class template
 * @return std::true_type if `Ts` is a base class template for `C`,
 * std::false_type otherwise
 */
template<typename Ts, template<typename...> class C>
using is_base_of_template = decltype(internal::is_base_of_template_impl<C>(std::declval<
    Ts *>()));

} // kodlab::type_traits
