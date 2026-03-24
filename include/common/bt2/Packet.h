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

#include <babeltrace2/trace-ir/packet.h>

#include "Object.h"


namespace bt2 {

class Field;

class Packet : public Object<bt_packet> {
public:
	Field* getContext() noexcept {
		return reinterpret_cast<Field*>( bt_packet_borrow_context_field( me() ) );
	}

	[[nodiscard]] const Field* getContext() const noexcept {
		return reinterpret_cast<const Field*>( bt_packet_borrow_context_field_const( me() ) );
	}

	Stream* getStream() noexcept {
		return reinterpret_cast<Stream*>( bt_packet_borrow_stream( me() ) );
	}

	[[nodiscard]] const Stream* getStream() const noexcept {
		return reinterpret_cast<const Stream*>( bt_packet_borrow_stream_const( me() ) );
	}
};

}  // namespace bt2
