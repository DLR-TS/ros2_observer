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

#include <babeltrace2/trace-ir/stream.h>

#include "Object.h"


namespace bt2 {

class StreamClass;
class Trace;
class Value;

class Stream : public Object<bt_stream> {
public:
	static Stream* create( StreamClass* streamClass, Trace* trace ) {
		auto* s = reinterpret_cast<bt_stream_class*>( streamClass );
		auto* t = reinterpret_cast<bt_trace*>( trace );
		return reinterpret_cast<Stream*>( bt_stream_create( s, t ) );
	}


	[[nodiscard]] uint64_t getId() const { return bt_stream_get_id( me() ); }


	StreamClass* getStreamClass() noexcept {
		return reinterpret_cast<StreamClass*>( bt_stream_borrow_class( me() ) );
	}

	[[nodiscard]] const StreamClass* getStreamClass() const noexcept {
		return reinterpret_cast<const StreamClass*>( bt_stream_borrow_class_const( me() ) );
	}


	Trace* getTrace() noexcept {
		return reinterpret_cast<Trace*>( bt_stream_borrow_trace( me() ) );
	}

	[[nodiscard]] const Trace* getTrace() const noexcept {
		return reinterpret_cast<const Trace*>( bt_stream_borrow_trace_const( me() ) );
	}


	Value* getUserAttributes() noexcept {
		return reinterpret_cast<Value*>( bt_stream_borrow_user_attributes( me() ) );
	}

	[[nodiscard]] const Value* getUserAttributes() const noexcept {
		return reinterpret_cast<const Value*>( bt_stream_borrow_user_attributes_const( me() ) );
	}

	void putRef() { bt_stream_put_ref( me() ); }
};

}  // namespace bt2
