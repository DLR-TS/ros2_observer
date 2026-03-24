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

#include <babeltrace2/trace-ir/trace.h>

#include "Object.h"


namespace bt2 {

class TraceClass;
class Value;

class Trace : public Object<bt_trace> {
public:
	static Trace* create( TraceClass* traceClass ) {
		auto* t = reinterpret_cast<bt_trace_class*>( traceClass );
		return reinterpret_cast<Trace*>( bt_trace_create( t ) );
	}

	const Value* getEnvironmentValueByName( const char* name ) const noexcept {
		const auto* value = bt_trace_borrow_environment_entry_value_by_name_const( me(), name );
		return reinterpret_cast<const Value*>( value );
	}

	void putRef() { bt_trace_put_ref( me() ); }
};

}  // namespace bt2
