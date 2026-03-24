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

#include "Message.h"


namespace bt2 {

class ClockSnapshot;
class Stream;

class StreamEndMessage : public Message {
public:
	[[nodiscard]] const ClockSnapshot* getDefaultClockSnapshot() const noexcept {
		const bt_clock_snapshot* clockSnapshot;
		bt_message_stream_end_borrow_default_clock_snapshot_const( me(), &clockSnapshot );
		return reinterpret_cast<const ClockSnapshot*>( clockSnapshot );
	}

	void setDefaultClockSnapshot( uint64_t value ) {
		bt_message_stream_end_set_default_clock_snapshot( me(), value );
	}

	Stream* getStream() noexcept {
		return reinterpret_cast<Stream*>( bt_message_stream_end_borrow_stream( me() ) );
	}

	[[nodiscard]] const Stream* getStream() const noexcept {
		return reinterpret_cast<const Stream*>( bt_message_stream_end_borrow_stream_const( me() ) );
	}
};

}  // namespace bt2
