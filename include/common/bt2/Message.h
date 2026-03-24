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

#include <babeltrace2/graph/message.h>

#include "Object.h"


namespace bt2 {

class ClockSnapshot;

class Message : public Object<bt_message> {
public:
	[[nodiscard]] bt_message_type getType() const noexcept { return bt_message_get_type( me() ); }

	void putRef() const noexcept { bt_message_put_ref( me() ); }
};

}  // namespace bt2
