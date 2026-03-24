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

#include <babeltrace2/trace-ir/event.h>

#include "Field.h"
#include "Object.h"


namespace bt2 {

class EventClass;
class Packet;
class Stream;

class Event : public Object<bt_event> {
public:
	Field* getCommonContextField() noexcept {
		return reinterpret_cast<Field*>( bt_event_borrow_common_context_field( me() ) );
	}

	[[nodiscard]] const Field* getCommonContextField() const noexcept {
		return reinterpret_cast<const Field*>( bt_event_borrow_common_context_field_const( me() ) );
	}

	EventClass* getEventClass() noexcept {
		return reinterpret_cast<EventClass*>( bt_event_borrow_class( me() ) );
	}

	[[nodiscard]] const EventClass* getEventClass() const noexcept {
		return reinterpret_cast<const EventClass*>( bt_event_borrow_class_const( me() ) );
	}

	Packet* getPacket() noexcept {
		return reinterpret_cast<Packet*>( bt_event_borrow_packet( me() ) );
	}

	[[nodiscard]] const Packet* getPacket() const noexcept {
		return reinterpret_cast<const Packet*>( bt_event_borrow_packet_const( me() ) );
	}

	Field* getPayloadField() noexcept {
		return reinterpret_cast<Field*>( bt_event_borrow_payload_field( me() ) );
	}

	[[nodiscard]] const Field* getPayloadField() const noexcept {
		return reinterpret_cast<const Field*>( bt_event_borrow_payload_field_const( me() ) );
	}

	Stream* getStream() noexcept {
		return reinterpret_cast<Stream*>( bt_event_borrow_stream( me() ) );
	}
	[[nodiscard]] const Stream* getStream() const noexcept {
		return reinterpret_cast<const Stream*>( bt_event_borrow_stream_const( me() ) );
	}

	Field* getSpecificContext() noexcept {
		return reinterpret_cast<Field*>( bt_event_borrow_specific_context_field( me() ) );
	}

	[[nodiscard]] const Field* getSpecificContext() const noexcept {
		return reinterpret_cast<const Field*>(
			bt_event_borrow_specific_context_field_const( me() ) );
	}


protected:
	template<typename R>
	[[nodiscard]] auto getPayloadFieldValueByName( const char* name ) const noexcept {
		const auto* payload = getPayloadField();
		const auto* field = payload->getFieldByName( name );
		return static_cast<R>( *field );
	}

	template<typename T>
	void setPayloadFieldValueByName( const char* name, T value ) noexcept {
		auto* payload = getPayloadField();
		auto* field = payload->getFieldByName( name );
		*field = value;
	}
};

}  // namespace bt2
