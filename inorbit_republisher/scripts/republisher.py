#!/usr/bin/env python

# Copyright 2021 InOrbit, Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# ROS to InOrbit republisher node sample
#
# It uses a YAML-based configuration to map between arbitrary
# ROS topics into InOrbit key/value custom data topics.

import json
import rospy
import genpy
import yaml
from std_msgs.msg import String
from roslib.message import get_message_class
from operator import attrgetter

# Types of mappings allowed
MAPPING_TYPE_SINGLE_FIELD = "single_field"
MAPPING_TYPE_ARRAY_OF_FIELDS = "array_of_fields"
MAPPING_TYPE_JSON_OF_FIELDS = "json_of_fields"

"""
Main node entry point.

Currently a simple function that does everything.
We can later turn it into a class and divide in proper methods
as complexity increases.
"""
def main():
    # Start the ROS node
    rospy.init_node('inorbit_republisher', anonymous=True)

    # Read republisher configuration from the 'config' or 'config_file' parameter
    # TODO(adamantivm) Error handling and schema checking
    if rospy.has_param('~config_file'):
        config_file = rospy.get_param('~config_file')
        rospy.loginfo("Using config from config file: {}".format(config_file))
        config_yaml = open(config_file, "r")
    elif rospy.has_param('~config'):
        config_yaml = rospy.get_param('~config')
        rospy.loginfo("Using config from parameter server")
    config = yaml.safe_load(config_yaml)

    # Go through republisher configurations
    # For each of them: create a publisher if necessary - only one per InOrbit
    # custom data field - and a matching subscriber to receive and republish
    # the desired fields.

    # Dictionary of publisher instances by topic name
    pubs = {}

    # Dictionary of subscriber instances by topic name
    subs = {}

    for repub in config['republishers']:

        # Load subscriber message type
        msg_class = get_message_class(repub['msg_type'])
        if msg_class is None:
            rospy.logwarn('Failed to load msg class for {}'.format(repub['msg_type']))
            continue

        # Create publisher for each new seen outgoing topic
        for mapping in repub['mappings']:
            out_topic = mapping['out']['topic']
            if not out_topic in pubs:
                pubs[out_topic] = rospy.Publisher(out_topic, String, queue_size=20)
            mapping['attrgetter'] = attrgetter(mapping['field'])

        # Prepare callback to relay messages through InOrbit custom data
        def callback(msg, repub=repub):

            for mapping in repub['mappings']:
                key = mapping['out']['key']
                val = ""
                mapping_type = mapping.get('mapping_type', MAPPING_TYPE_SINGLE_FIELD)
                topic = mapping['out']['topic']

                if mapping_type == MAPPING_TYPE_SINGLE_FIELD:
                    val = extract_value(msg, attrgetter(mapping['field']))

                elif mapping_type == MAPPING_TYPE_ARRAY_OF_FIELDS:
                    field = extract_value(msg, attrgetter(mapping['field']))
                    val = process_array(field, mapping)

                elif mapping_type == MAPPING_TYPE_JSON_OF_FIELDS:
                    try:
                        val = extract_values_as_dict(msg, mapping)
                        val = json.dumps(val)
                    except TypeError as e:
                        rospy.logwarn("Failed to serialize message: %s", e)

                pubs[topic].publish("{}={}".format(key, val))

        in_topic = repub['topic']

        # subscribe
        subs[in_topic] = rospy.Subscriber(in_topic, msg_class, callback)

    rospy.loginfo('Republisher started')
    rospy.spin()
    rospy.loginfo('Republisher shutting down')

    # Disconnect subs and pubs
    for sub in subs.values():
        sub.unregister()

    for pub in pubs.values():
        pub.unregister()

"""
Extracts a value from the given message using the provided getter function
"""
def extract_value(msg, getter_fn):
    # TODO(adamantivm) Graceful handling of missing values to extract
    # TODO(adamantivm) Allow serialization of complex values
    val = getter_fn(msg)
    return val

"""
Extracts several values from a given nested msg field and returns a dictionary of
<field, value> elements
"""
def extract_values_as_dict(msg, mapping):
    values = {}
    base_getter_fn = attrgetter(mapping['field'])
    base_value = base_getter_fn(msg)
    fields = mapping.get('mapping_options', {}).get('fields')
    for field in fields:
        getter_fn = attrgetter(field)
        try:
            val = getter_fn(base_value)
            # genpy.Time values can't be serialized into JSON. convert them to seconds
            # TODO(diegobatt): Catch other datatypes
            if isinstance(val, genpy.Time):
                val = val.to_sec()
            # TODO(diegobatt): Make it possible to use a different key than the field
            values[field] = val
        except AttributeError as e:
            rospy.logwarn('Couldn\'t get attribute %s: %s', field, e)
    return values

"""
Processes a given array field from the ROS message and:
    - Filters it using the 'filter' function (if provided)
    - For each element, it gets the set of keys defined by array_fields
    - Returns a key/value with the value being a json string containing
      the resulting array of objects.

Note that the array fields to retrieve should have a String value in order to
serialize them properly.
"""
def process_array(field, mapping):
    # Output array of objects
    values = {
        'data': []
    }

    filter_fn = mapping.get('mapping_options', {}).get('filter')
    if filter_fn:
        # Apply the filter function if any
        filtered_array = list(filter(eval(filter_fn), field))
    else:
        filtered_array = field

    # Get only the array_fields specified. If none was specified, return the whole array
    # TODO(FlorGrosso): check that the array fields are Strings and discard those  which
    # are not.
    if 'fields' in mapping.get('mapping_options', {}):
        fields = mapping['mapping_options']['fields']
        values['data'] = [{f: extract_value(elem, attrgetter(f)) for f in fields} for elem in filtered_array]
    else:
        values['data'] = filtered_array

    return json.dumps(values)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass