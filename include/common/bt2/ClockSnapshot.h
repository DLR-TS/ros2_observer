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

#include <optional>

#include <babeltrace2/trace-ir/clock-snapshot.h>

#include "Object.h"


namespace bt2 {

class ClockSnapshot : public Object<bt_clock_snapshot> {
public:
	[[nodiscard]] uint64_t getValue() const noexcept { return bt_clock_snapshot_get_value( me() ); }

	[[nodiscard]] std::optional<int64_t> getNsFromOrigin() const noexcept {
		int64_t result;
		if ( auto status = bt_clock_snapshot_get_ns_from_origin( me(), &result );
			 status == BT_CLOCK_SNAPSHOT_GET_NS_FROM_ORIGIN_STATUS_OK ) {
			return result;
		}

		return std::nullopt;
	}
};

}  // namespace bt2
