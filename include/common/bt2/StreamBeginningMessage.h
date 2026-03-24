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

class StreamBeginningMessage : public Message {
public:
	void setDefaultClockSnapshot( uint64_t value ) {
		bt_message_stream_beginning_set_default_clock_snapshot( me(), value );
	}

	Stream* getStream() noexcept {
		return reinterpret_cast<Stream*>( bt_message_stream_beginning_borrow_stream( me() ) );
	}

	[[nodiscard]] const Stream* getStream() const noexcept {
		const auto* stream = bt_message_stream_beginning_borrow_stream_const( me() );
		return reinterpret_cast<const Stream*>( stream );
	}
};

}  // namespace bt2
