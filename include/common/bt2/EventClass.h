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

#include <babeltrace2/trace-ir/event-class.h>

#include "Object.h"


namespace bt2 {

class FieldClass;
class StreamClass;

class EventClass : public Object<bt_event_class> {
public:
	static EventClass* create( StreamClass* streamClass ) noexcept {
		auto* s = reinterpret_cast<bt_stream_class*>( streamClass );
		return reinterpret_cast<EventClass*>( bt_event_class_create( s ) );
	}

	static EventClass* createWithId( StreamClass* streamClass, uint64_t id ) noexcept {
		auto* s = reinterpret_cast<bt_stream_class*>( streamClass );
		return reinterpret_cast<EventClass*>( bt_event_class_create_with_id( s, id ) );
	}

	[[nodiscard]] uint64_t getId() const noexcept { return bt_event_class_get_id( me() ); }

	[[nodiscard]] const char* getName() const noexcept { return bt_event_class_get_name( me() ); }
	void setName( const char* name ) noexcept {
		// TODO: check return value
		bt_event_class_set_name( me(), name );
	}

	void setPayloadFieldClass( FieldClass* fieldClass ) {
		auto* f = reinterpret_cast<bt_field_class*>( fieldClass );
		bt_event_class_set_payload_field_class( me(), f );
	}

	void putRef() const noexcept { bt_event_class_put_ref( me() ); }
};

}  // namespace bt2
