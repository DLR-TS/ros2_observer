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

class Event;

class EventMessage : public Message {
public:
	[[nodiscard]] const ClockSnapshot* getDefaultClockSnapshot() const noexcept {
		const auto* clockSnapshot = bt_message_event_borrow_default_clock_snapshot_const( me() );
		return reinterpret_cast<const ClockSnapshot*>( clockSnapshot );
	}


	Event* getEvent() noexcept {
		return reinterpret_cast<Event*>( bt_message_event_borrow_event( me() ) );
	}

	[[nodiscard]] const Event* getEvent() const noexcept {
		return reinterpret_cast<const Event*>( bt_message_event_borrow_event_const( me() ) );
	}
};

}  // namespace bt2
