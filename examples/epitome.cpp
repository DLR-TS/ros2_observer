/*
 * The file is based on the sample sink component found at
 * https://babeltrace.org/docs/v2.1/libbabeltrace2/example-simple-sink-cmp-cls.html.
 * It was modified to demonstrate the use of the C++ wrapper classes.
 *
 * Copyright (c) 2025 INCHRON AG <info@inchron.com>
 *
 * This program and the accompanying materials are made available under the
 * terms of the Creative Commons Attribution-ShareAlike 4.0 International which
 * is available at https://creativecommons.org/licenses/by-sa/4.0/
 *
 * SPDX-License-Identifier: CC-BY-SA-4.0
 */

#include <iostream>
#include <memory>

#include <babeltrace2/babeltrace.h>

#include <common/bt2/Event.h>
#include <common/bt2/EventClass.h>
#include <common/bt2/EventMessage.h>
#include <common/bt2/Message.h>
#include <common/bt2/MessageIterator.h>
#include <common/bt2/Sink.h>


/* Sink component's private data */
struct Data {
	/* Upstream message iterator (owned by this) */
	bt2::MessageIterator* message_iterator;

	/* Current event message index */
	uint64_t index;
};

/*
 * Initializes the sink component.
 */
static bt_component_class_initialize_method_status epitome_out_initialize(
	bt_self_component_sink* self_component_sink,
	bt_self_component_sink_configuration* configuration, const bt_value* params,
	void* initialize_method_data ) {
	auto* sink = reinterpret_cast<bt2::Sink*>( self_component_sink );
	/* Allocate a private data structure */
	auto epitome_out = std::make_unique<Data>();

	/* Initialize the first event message's index */
	epitome_out->index = 1;

	/* Set the component's user data to our private data structure */
	sink->setData( epitome_out.release() );

	/*
     * Add an input port named `in` to the sink component.
     *
     * This is needed so that this sink component can be connected to a
     * filter or a source component. With a connected upstream
     * component, this sink component can create a message iterator
     * to consume messages.
     */
	sink->addInputPort( "in" );

	return BT_COMPONENT_CLASS_INITIALIZE_METHOD_STATUS_OK;
}

/*
 * Finalizes the sink component.
 */
static void epitome_out_finalize( bt_self_component_sink* self_component_sink ) {
	auto* sink = reinterpret_cast<bt2::Sink*>( self_component_sink );

	/* Retrieve our private data from the component's user data */
	std::unique_ptr<Data> epitome_out{ sink->getData<Data>() };

	/* Free the allocated structure */
	/* The destructor of std::unique_ptr does that for us */
}

/*
 * Called when the trace processing graph containing the sink component
 * is configured.
 *
 * This is where we can create our upstream message iterator.
 */
static bt_component_class_sink_graph_is_configured_method_status epitome_out_graph_is_configured(
	bt_self_component_sink* self_component_sink ) {
	auto* sink = reinterpret_cast<bt2::Sink*>( self_component_sink );

	/* Retrieve our private data from the component's user data */
	auto* epitome_out = sink->getData<Data>();

	/* Borrow our unique port */
	auto* in_port = sink->getInputPortByIndex( 0 );

	/* Create the upstream message iterator */
	epitome_out->message_iterator = bt2::MessageIterator::create( sink, in_port );

	return BT_COMPONENT_CLASS_SINK_GRAPH_IS_CONFIGURED_METHOD_STATUS_OK;
}

/*
 * Prints a line for `message`, if it's an event message, to the
 * standard output.
 */
static void print_message( Data* epitome_out, const bt2::Message* msg ) {
	/* Discard if it's not an event message */
	if ( msg->getType() != BT_MESSAGE_TYPE_EVENT ) {
		return;
	}

	/* The message is of type event */
	const auto* message = reinterpret_cast<const bt2::EventMessage*>( msg );

	/* Borrow the event message's event and its class */
	const auto* event = message->getEvent();
	const auto* event_class = event->getEventClass();

	/* Get the number of payload field members */
	const auto* payload_field = event->getPayloadField();

	/* The class bt2::Field does not have a function to get its class, so revert to the C API */
	const auto* pf = reinterpret_cast<const bt_field*>( payload_field );
	uint64_t member_count =
		bt_field_class_structure_get_member_count( bt_field_borrow_class_const( pf ) );

	/* Write a corresponding line to the standard output */
	std::cout << epitome_out->index << ": " << event_class->getName() << " (" << member_count
			  << " payload member" << ( member_count == 1 ? "" : "s" ) << ")\n";

	/* Increment the current event message's index */
	epitome_out->index++;
}

/*
 * Consumes a batch of messages and writes the corresponding lines to
 * the standard output.
 */
bt_component_class_sink_consume_method_status epitome_out_consume(
	bt_self_component_sink* self_component_sink ) {
	auto* sink = reinterpret_cast<bt2::Sink*>( self_component_sink );

	/* Retrieve our private data from the component's user data */
	auto* epitome_out = sink->getData<Data>();

	/* Consume a batch of messages from the upstream message iterator */
	auto&& [messages, next_status] = epitome_out->message_iterator->next();

	switch ( next_status ) {
	case BT_MESSAGE_ITERATOR_NEXT_STATUS_END:
		/* End of iteration: put the message iterator's reference */
		epitome_out->message_iterator->putRef();
		return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_END;

	case BT_MESSAGE_ITERATOR_NEXT_STATUS_AGAIN:
		return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_AGAIN;

	case BT_MESSAGE_ITERATOR_NEXT_STATUS_MEMORY_ERROR:
		return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_MEMORY_ERROR;

	case BT_MESSAGE_ITERATOR_NEXT_STATUS_ERROR:
		return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_ERROR;

	default:
		break;
	}

	/* For each consumed message */
	for ( const auto* message : messages ) {
		/* Print line for current message if it's an event message */
		print_message( epitome_out, message );

		/* Put this message's reference */
		message->putRef();
	}

	return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_OK;
}

/* Mandatory */
BT_PLUGIN_MODULE();

/* Define the `epitome` plugin */
BT_PLUGIN( epitome );

/* Define the `output` sink component class */
BT_PLUGIN_SINK_COMPONENT_CLASS( output, epitome_out_consume );

/* Set some of the `output` sink component class's optional methods */
BT_PLUGIN_SINK_COMPONENT_CLASS_INITIALIZE_METHOD( output, epitome_out_initialize );
BT_PLUGIN_SINK_COMPONENT_CLASS_FINALIZE_METHOD( output, epitome_out_finalize );
BT_PLUGIN_SINK_COMPONENT_CLASS_GRAPH_IS_CONFIGURED_METHOD( output,
														   epitome_out_graph_is_configured );
