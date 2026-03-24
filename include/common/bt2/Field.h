/*
 * Copyright (c) 2025 INCHRON AG <info@inchron.com>
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0/
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include <cassert>
#include <type_traits>

#include <babeltrace2/trace-ir/field.h>

#include "Object.h"


namespace bt2 {

class Field : public Object<bt_field> {
public:
	template<typename T,
			 std::enable_if_t<std::is_integral_v<T> && std::is_unsigned_v<T>, bool> = true>
	explicit operator T() const noexcept {
		return static_cast<T>( bt_field_integer_unsigned_get_value( me() ) );
	}

	template<typename T,
			 std::enable_if_t<std::is_integral_v<T> && std::is_signed_v<T>, bool> = true>
	explicit operator T() const noexcept {
		return static_cast<T>( bt_field_integer_signed_get_value( me() ) );
	}

	template<typename T, std::enable_if_t<std::is_enum_v<T>, bool> = true>
	explicit operator T() const noexcept {
		return static_cast<T>( static_cast<std::underlying_type_t<T>>( *this ) );
	}

	template<typename T, std::enable_if_t<std::is_convertible_v<const char*, T>, bool> = true>
	explicit operator T() const noexcept {
		return bt_field_string_get_value( me() );
	}


	template<typename T,
			 std::enable_if_t<std::is_integral_v<T> && std::is_unsigned_v<T>, bool> = true>
	Field& operator=( T value ) noexcept {
		bt_field_integer_unsigned_set_value( me(), value );
		return *this;
	}

	template<typename T,
			 std::enable_if_t<std::is_integral_v<T> && std::is_signed_v<T>, bool> = true>
	Field& operator=( T value ) noexcept {
		bt_field_integer_signed_set_value( me(), value );
		return *this;
	}

	template<typename T, std::enable_if_t<std::is_enum_v<T>, bool> = true>
	Field& operator=( T value ) noexcept {
		return *this = static_cast<std::underlying_type_t<T>>( value );
	}


	Field* getFieldByIndex( uint64_t index ) noexcept {
		return reinterpret_cast<Field*>(
			bt_field_structure_borrow_member_field_by_index( me(), index ) );
	}

	[[nodiscard]] const Field* getFieldByIndex( uint64_t index ) const noexcept {
		return reinterpret_cast<const Field*>(
			bt_field_structure_borrow_member_field_by_index_const( me(), index ) );
	}

	Field* getFieldByName( const char* name ) noexcept {
		return reinterpret_cast<Field*>(
			bt_field_structure_borrow_member_field_by_name( me(), name ) );
	}

	[[nodiscard]] const Field* getFieldByName( const char* name ) const noexcept {
		return reinterpret_cast<const Field*>(
			bt_field_structure_borrow_member_field_by_name_const( me(), name ) );
	}


	template<typename T>
	T getValue() {
		return static_cast<T>( *this );
	}

	template<typename T>
	void setValue( T value ) {
		return *this = value;
	}
};

}  // namespace bt2
