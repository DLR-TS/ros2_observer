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
#include <string_view>
#if __has_include( <span>)
#	include <span>
#endif
#ifndef __cpp_lib_span
#	include <boost/core/span.hpp>
#endif

#include <babeltrace2/graph/message-iterator.h>

#include "Exception.h"
#include "InputPort.h"
#include "Object.h"


namespace bt2 {

class Component;
class EventClass;
class EventMessage;
class Message;
#if __cpp_lib_span
using Messages = std::span<const Message*>;
#else
using Messages = boost::span<const Message*>;
#endif
class OutputPort;
class Sink;
class Stream;
class StreamBeginningMessage;
class StreamEndMessage;

class MessageIterator : public Object<bt_message_iterator>,
						public Object<bt_self_message_iterator> {
public:
	static MessageIterator* create( Sink* sink, InputPort* input ) {
		auto* s = reinterpret_cast<bt_self_component_sink*>( sink );
		auto* port = reinterpret_cast<bt_self_component_port_input*>( input );
		bt_message_iterator* iterator;
		const auto status = bt_message_iterator_create_from_sink_component( s, port, &iterator );
		constexpr uint64_t lineNumber = __LINE__ - 1;
		checkStatus( status, lineNumber );
		return reinterpret_cast<MessageIterator*>( iterator );
	}

	std::pair<Messages, bt_message_iterator_next_status> next() {
		auto* iterator = Object<bt_message_iterator>::me();
		bt_message_array_const messages;
		uint64_t count;
		auto status = bt_message_iterator_next( iterator, &messages, &count );
		if ( status != BT_MESSAGE_ITERATOR_NEXT_STATUS_OK ) {
			count = 0;
		}
		auto* m = reinterpret_cast<const Message**>( messages );
		return { { m, count }, status };
	}

	void putRef() { bt_message_iterator_put_ref( Object<bt_message_iterator>::me() ); }

protected:
	template<typename T>
	void setData( T* data ) {
		auto* iterator = Object<bt_self_message_iterator>::me();
		bt_self_message_iterator_set_data( iterator, data );
	}

	template<typename T>
	T* getData() const {
		const auto* iterator = Object<bt_self_message_iterator>::me();
		return reinterpret_cast<T*>( bt_self_message_iterator_get_data( iterator ) );
	}

	Component* getComponent() {
		auto* iterator = Object<bt_self_message_iterator>::me();
		auto* component = bt_self_message_iterator_borrow_component( iterator );
		return reinterpret_cast<Component*>( component );
	}


	OutputPort* getOutputPort() {
		auto* iterator = Object<bt_self_message_iterator>::me();
		auto* outputPort = bt_self_message_iterator_borrow_port( iterator );
		return reinterpret_cast<OutputPort*>( outputPort );
	}

	MessageIterator* create( InputPort* input ) {
		auto* iterator = Object<bt_self_message_iterator>::me();
		auto* port = reinterpret_cast<bt_self_component_port_input*>( input );
		bt_message_iterator* it;
		const auto status = bt_message_iterator_create_from_message_iterator( iterator, port, &it );
		constexpr uint64_t lineNumber = __LINE__ - 1;
		checkStatus( status, lineNumber );
		return reinterpret_cast<MessageIterator*>( it );
	}

	EventMessage* createEventMessageWithClockSnapshot( const EventClass* eventClass,
													   const Stream* stream,
													   uint64_t clockSnapshotValue ) {
		auto* iterator = Object<bt_self_message_iterator>::me();
		const auto* ec = reinterpret_cast<const bt_event_class*>( eventClass );
		const auto* s = reinterpret_cast<const bt_stream*>( stream );
		auto* message = bt_message_event_create_with_default_clock_snapshot( iterator, ec, s,
																			 clockSnapshotValue );
		return reinterpret_cast<EventMessage*>( message );
	}

	StreamBeginningMessage* createStreamBeginning(
		Stream* stream, std::optional<uint64_t> clockSnapshotValue = std::nullopt ) {
		auto* iterator = Object<bt_self_message_iterator>::me();
		auto* s = reinterpret_cast<bt_stream*>( stream );
		auto* message = bt_message_stream_beginning_create( iterator, s );
		if ( clockSnapshotValue.has_value() ) {
			bt_message_stream_beginning_set_default_clock_snapshot( message,
																	clockSnapshotValue.value() );
		}
		return reinterpret_cast<StreamBeginningMessage*>( message );
	}

	StreamEndMessage* createStreamEnd( Stream* stream,
									   std::optional<uint64_t> clockSnapshotValue = std::nullopt ) {
		auto* iterator = Object<bt_self_message_iterator>::me();
		auto* s = reinterpret_cast<bt_stream*>( stream );
		auto* message = bt_message_stream_end_create( iterator, s );
		if ( clockSnapshotValue.has_value() ) {
			bt_message_stream_end_set_default_clock_snapshot( message, clockSnapshotValue.value() );
		}
		return reinterpret_cast<StreamEndMessage*>( message );
	}


	void appendErrorCause( const Exception& e ) {
		auto* iterator = Object<bt_self_message_iterator>::me();
		bt_current_thread_error_append_cause_from_message_iterator(
			iterator, e.fileName(), e.lineNumber(), "%s", e.what() );
	}

private:
	static void checkStatus( const bt_message_iterator_create_from_message_iterator_status status,
							 const uint64_t lineNumber ) {
		static constexpr std::string_view filename{ __FILE__ };
		static constexpr std::string_view errorMessage{
			"Creating message iterator caused an error" };

		switch ( status ) {
		case BT_MESSAGE_ITERATOR_CREATE_FROM_MESSAGE_ITERATOR_STATUS_OK:
			return;

		case BT_MESSAGE_ITERATOR_CREATE_FROM_MESSAGE_ITERATOR_STATUS_MEMORY_ERROR:
			throw MemoryError{ filename.data(), lineNumber, errorMessage.data() };

		case BT_MESSAGE_ITERATOR_CREATE_FROM_MESSAGE_ITERATOR_STATUS_ERROR:
		default:
			throw Error{ filename.data(), lineNumber, errorMessage.data() };
		}
	}

	static void checkStatus( const bt_message_iterator_create_from_sink_component_status status,
							 const uint64_t lineNumber ) {
		static constexpr std::string_view filename{ __FILE__ };
		static constexpr std::string_view errorMessage{
			"Creating message iterator caused an error" };
		switch ( status ) {
		case BT_MESSAGE_ITERATOR_CREATE_FROM_SINK_COMPONENT_STATUS_OK:
			return;

		case BT_MESSAGE_ITERATOR_CREATE_FROM_SINK_COMPONENT_STATUS_MEMORY_ERROR:
			throw MemoryError{ filename.data(), lineNumber, errorMessage.data() };

		case BT_MESSAGE_ITERATOR_CREATE_FROM_SINK_COMPONENT_STATUS_ERROR:
		default:
			throw Error{ filename.data(), lineNumber, errorMessage.data() };
		}
	}
};

}  // namespace bt2
