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

#include <babeltrace2/value.h>

#include "Object.h"


namespace bt2 {

class Value : public Object<bt_value> {
public:
	template<typename T, std::enable_if_t<std::is_convertible_v<const char*, T>, bool> = true>
	explicit operator T() const noexcept {
		return bt_value_string_get( me() );
	}


	Value* getMapEntry( const char* key ) noexcept {
		return reinterpret_cast<Value*>( bt_value_map_borrow_entry_value( me(), key ) );
	}

	const Value* getMapEntry( const char* key ) const noexcept {
		return reinterpret_cast<const Value*>( bt_value_map_borrow_entry_value_const( me(), key ) );
	}

	void insertMapEntry( const char* key, const char* value ) noexcept {
		bt_value_map_insert_string_entry( me(), key, value );
	}
};

}  // namespace bt2
