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

#include <cstdint>
#include <exception>


namespace bt2 {

class Exception : public std::exception {
public:
	Exception( const char* fileName, uint64_t lineNumber, const char* message )
		: _fileName( fileName ), _lineNumber( lineNumber ), _message( message ) {}

	[[nodiscard]] const char* what() const noexcept override { return _message; }

	[[nodiscard]] const char* fileName() const { return _fileName; }
	[[nodiscard]] uint64_t lineNumber() const { return _lineNumber; }

private:
	const char* _fileName{ nullptr };
	uint64_t _lineNumber{};
	const char* _message{ nullptr };
};

class Error final : public Exception {
public:
	using Exception::Exception;
};
class MemoryError final : public Exception {
	using Exception::Exception;
};

}  // namespace bt2
