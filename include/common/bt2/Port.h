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

#include <babeltrace2/graph/self-component-port.h>

#include "Object.h"


namespace bt2 {

class Component;

class Port : public Object<bt_self_component_port> {
public:
	Component* getComponent() noexcept {
		auto* component = bt_self_component_port_borrow_component( me() );
		return reinterpret_cast<Component*>( component );
	}

	template<typename T>
	T* getData() const noexcept {
		auto* data = bt_self_component_port_get_data( me() );
		return reinterpret_cast<T*>( data );
	}
};

}  // namespace bt2
