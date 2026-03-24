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

#include <babeltrace2/trace-ir/trace-class.h>

#include "Object.h"


namespace bt2 {

class Component;
class StreamClass;

class TraceClass : public Object<bt_trace_class> {
public:
	static TraceClass* create( Component* component ) noexcept {
		auto* c = reinterpret_cast<bt_self_component*>( component );
		return reinterpret_cast<TraceClass*>( bt_trace_class_create( c ) );
	}


	StreamClass* getStreamClassByIndex( uint64_t index ) noexcept {
		auto* streamClass = bt_trace_class_borrow_stream_class_by_index( me(), index );
		return reinterpret_cast<StreamClass*>( streamClass );
	}

	[[nodiscard]] const StreamClass* getStreamClassByIndex( uint64_t index ) const noexcept {
		const auto* streamClass = bt_trace_class_borrow_stream_class_by_index_const( me(), index );
		return reinterpret_cast<const StreamClass*>( streamClass );
	}

	void putRef() const noexcept { bt_trace_class_put_ref( me() ); }
};

}  // namespace bt2
