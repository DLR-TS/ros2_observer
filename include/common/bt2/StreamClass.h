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

#include <babeltrace2/trace-ir/stream-class.h>

#include "Object.h"


namespace bt2 {

class ClockClass;
class EventClass;
class TraceClass;

class StreamClass : public Object<bt_stream_class> {
public:
	static StreamClass* create( TraceClass* traceClass ) noexcept {
		auto* t = reinterpret_cast<bt_trace_class*>( traceClass );
		return reinterpret_cast<StreamClass*>( bt_stream_class_create( t ) );
	}


	void setDefaultClockClass( ClockClass* clockClass ) noexcept {
		auto* c = reinterpret_cast<bt_clock_class*>( clockClass );
		bt_stream_class_set_default_clock_class( me(), c );
	}


	EventClass* getEventClassById( uint64_t id ) noexcept {
		auto* streamClass = bt_stream_class_borrow_event_class_by_id( me(), id );
		return reinterpret_cast<EventClass*>( streamClass );
	}

	[[nodiscard]] const EventClass* getEventClassById( uint64_t id ) const noexcept {
		const auto* eventClass = bt_stream_class_borrow_event_class_by_id_const( me(), id );
		return reinterpret_cast<const EventClass*>( eventClass );
	}


	TraceClass* getTraceClass() noexcept {
		return reinterpret_cast<TraceClass*>( bt_stream_class_borrow_trace_class( me() ) );
	}

	[[nodiscard]] const TraceClass* getTraceClass() const noexcept {
		const auto* traceClass = bt_stream_class_borrow_trace_class_const( me() );
		return reinterpret_cast<const TraceClass*>( traceClass );
	}


	void setAssignsAutomaticEventClassId( bt_bool assignsAutomaticEventClassId ) {
		bt_stream_class_set_assigns_automatic_event_class_id( me(), assignsAutomaticEventClassId );
	}

	void putRef() const noexcept { bt_stream_class_put_ref( me() ); }
};

}  // namespace bt2
