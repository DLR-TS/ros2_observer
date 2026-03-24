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

#include <babeltrace2/graph/self-component.h>

#include "Component.h"


namespace bt2 {

class InputPort;

class Sink : public Component,
			 public Object<bt_self_component_sink> {
public:
	InputPort* addInputPort( const char* name, void* data = nullptr ) noexcept {
		auto* sink = Object<bt_self_component_sink>::me();
		bt_self_component_port_input* port;
		const auto status = bt_self_component_sink_add_input_port( sink, name, data, &port );
		constexpr uint64_t lineNumber = __LINE__ - 1;
		checkStatus( status, lineNumber );
		return reinterpret_cast<InputPort*>( port );
	}

	InputPort* getInputPortByIndex( uint64_t index ) {
		auto* sink = Object<bt_self_component_sink>::me();
		auto* inputPort = bt_self_component_sink_borrow_input_port_by_index( sink, index );
		return reinterpret_cast<InputPort*>( inputPort );
	}
};

}  // namespace bt2