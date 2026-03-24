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


namespace bt2 {

template<typename T>
class Object {
public:
	// Any Object is used as a wrapper, so none of the special member functions shall be usable
	Object() = delete;
	Object( const Object& ) = delete;
	Object( Object&& ) = delete;
	Object& operator=( const Object& ) = delete;
	Object& operator=( Object&& ) = delete;
	~Object() = delete;

	explicit operator T&() noexcept { return *me(); }
	explicit operator const T&() const noexcept { return *me(); }


protected:
	T* me() { return reinterpret_cast<T*>( this ); }
	[[nodiscard]] const T* me() const { return reinterpret_cast<const T*>( this ); }
};

}  // namespace bt2
