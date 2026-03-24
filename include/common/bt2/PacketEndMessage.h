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
class Packet;

class PacketEndMessage : public Message {
public:
	[[nodiscard]] const ClockSnapshot* getDefaultClockSnapshot() const noexcept {
		const auto* cs = bt_message_packet_end_borrow_default_clock_snapshot_const( me() );
		return reinterpret_cast<const ClockSnapshot*>( cs );
	}


	Packet* getPacket() noexcept {
		return reinterpret_cast<Packet*>( bt_message_packet_end_borrow_packet( me() ) );
	}

	[[nodiscard]] const Packet* getPacket() const noexcept {
		return reinterpret_cast<const Packet*>( bt_message_packet_end_borrow_packet_const( me() ) );
	}
};

}  // namespace bt2
