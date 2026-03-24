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
class OutputPort;

class Filter : public Component,
			   public Object<bt_self_component_filter> {
public:
	InputPort* addInputPort( const char* name, void* data = nullptr ) noexcept {
		auto* filter = Object<bt_self_component_filter>::me();
		bt_self_component_port_input* port;
		const auto status = bt_self_component_filter_add_input_port( filter, name, data, &port );
		constexpr uint64_t lineNumber = __LINE__ - 1;
		checkStatus( status, lineNumber );
		return reinterpret_cast<InputPort*>( port );
	}

	OutputPort* addOutputPort( const char* name, void* data = nullptr ) noexcept {
		auto* filter = Object<bt_self_component_filter>::me();
		bt_self_component_port_output* port;
		const auto status = bt_self_component_filter_add_output_port( filter, name, data, &port );
		constexpr uint64_t lineNumber = __LINE__ - 1;
		checkStatus( status, lineNumber );
		return reinterpret_cast<OutputPort*>( port );
	}

	InputPort* getInputPortByIndex( uint64_t index ) noexcept {
		auto* filter = Object<bt_self_component_filter>::me();
		auto* port = bt_self_component_filter_borrow_input_port_by_index( filter, index );
		return reinterpret_cast<InputPort*>( port );
	}

	OutputPort* getOutputPortByIndex( uint64_t index ) noexcept {
		auto* filter = Object<bt_self_component_filter>::me();
		auto* port = bt_self_component_filter_borrow_output_port_by_index( filter, index );
		return reinterpret_cast<OutputPort*>( port );
	}
};

}  // namespace bt2
