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

#include "Object.h"
#include "Port.h"


namespace bt2 {

class OutputPort : public Port,
				   public Object<bt_self_component_port_output> {};

}  // namespace bt2
