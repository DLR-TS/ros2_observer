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

#include <string_view>

#include <babeltrace2/graph/self-component.h>

#include "Exception.h"
#include "Object.h"


namespace bt2 {

class Component : public Object<bt_self_component> {
public:
	template<typename T>
	void setData( T* data ) noexcept {
		bt_self_component_set_data( me(), data );
	}

	template<typename T>
	T* getData() const noexcept {
		return reinterpret_cast<T*>( bt_self_component_get_data( me() ) );
	}

protected:
	static void checkStatus( const bt_self_component_add_port_status status,
							 const uint64_t lineNumber ) {
		static constexpr std::string_view filename{ __FILE__ };
		static constexpr std::string_view errorMessage{ "Adding port caused an error" };

		switch ( status ) {
		case BT_SELF_COMPONENT_ADD_PORT_STATUS_OK:
			return;

		case BT_SELF_COMPONENT_ADD_PORT_STATUS_MEMORY_ERROR:
			throw MemoryError{ filename.data(), lineNumber, errorMessage.data() };

		case BT_SELF_COMPONENT_ADD_PORT_STATUS_ERROR:
		default:
			throw Error{ filename.data(), lineNumber, errorMessage.data() };
		}
	}

	void appendErrorCause( const Exception& e ) {
		bt_current_thread_error_append_cause_from_component( me(), e.fileName(), e.lineNumber(),
															 "%s", e.what() );
	}
};

}  // namespace bt2
