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

#include <climits>
#include <type_traits>

#include <babeltrace2/trace-ir/field-class.h>

#include "Object.h"


namespace bt2 {

class TraceClass;

class FieldClass : public Object<bt_field_class> {
public:
	template<typename T>
	static std::enable_if_t<std::is_integral_v<T> && std::is_unsigned_v<T>, FieldClass*> create(
		TraceClass* traceClass ) noexcept {
		auto* t = reinterpret_cast<bt_trace_class*>( traceClass );
		auto* fieldClass = bt_field_class_integer_unsigned_create( t );
		auto* fc = reinterpret_cast<FieldClass*>( fieldClass );
		fc->setFieldValueRange( CHAR_BIT * sizeof( T ) );
		return fc;
	}

	template<typename T>
	static std::enable_if_t<std::is_integral_v<T> && std::is_signed_v<T>, FieldClass*> create(
		TraceClass* traceClass ) noexcept {
		auto* t = reinterpret_cast<bt_trace_class*>( traceClass );
		auto* fieldClass = bt_field_class_integer_signed_create( t );
		auto* fc = reinterpret_cast<FieldClass*>( fieldClass );
		fc->setFieldValueRange( CHAR_BIT * sizeof( T ) );
		return fc;
	}

	template<typename T>
	static std::enable_if_t<std::is_enum_v<T>, FieldClass*> create(
		TraceClass* traceClass ) noexcept {
		return create<std::underlying_type_t<T>>( traceClass );
	}

	template<typename T>
	static std::enable_if_t<std::is_aggregate_v<T>, FieldClass*> create(
		TraceClass* traceClass ) noexcept {
		auto* t = reinterpret_cast<bt_trace_class*>( traceClass );
		return reinterpret_cast<FieldClass*>( bt_field_class_structure_create( t ) );
	}


	template<typename T>
	void appendStructureMember( TraceClass* traceClass, const char* memberName ) {
		auto* memberFieldClass = create<T>( traceClass );
		auto* mfc = reinterpret_cast<bt_field_class*>( memberFieldClass );
		bt_field_class_structure_append_member( me(), memberName, mfc );
		memberFieldClass->putRef();
	}

	void setFieldValueRange( uint64_t n ) {
		bt_field_class_integer_set_field_value_range( me(), n );
	}

	void putRef() const { bt_field_class_put_ref( me() ); }
};

}  // namespace bt2
