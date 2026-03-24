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

#include <babeltrace2/trace-ir/clock-class.h>

#include "Object.h"


namespace bt2 {

class Component;

class ClockClass : public Object<bt_clock_class> {
public:
	static ClockClass* create( Component* component ) noexcept {
		auto* c = reinterpret_cast<bt_self_component*>( component );
		auto* clockClass = bt_clock_class_create( c );
		return reinterpret_cast<ClockClass*>( clockClass );
	}

	void setFrequency( uint64_t frequency ) noexcept {
		bt_clock_class_set_frequency( me(), frequency );
	}

	void setOriginIsUnixEpoch( bt_bool originIsUnixEpoch ) noexcept {
		bt_clock_class_set_origin_is_unix_epoch( me(), originIsUnixEpoch );
	}

	void putRef() const noexcept { bt_clock_class_put_ref( me() ); }
};

}  // namespace bt2
