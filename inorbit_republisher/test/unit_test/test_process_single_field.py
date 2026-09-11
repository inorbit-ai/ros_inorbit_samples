#!/usr/bin/env python3
"""
Unit tests for process_single_field: the 'scale' mapping option and how it
interacts with 'filter'.
"""
import pytest

from inorbit_republisher.republisher import process_single_field


def mapping(**options):
    return {'mapping_options': options} if options else {}


def test_no_options_passes_value_through():
    assert process_single_field(0.898, mapping()) == 0.898


def test_scale_multiplies():
    # sensor_msgs/BatteryState.percentage is 0..1; InOrbit shows a percentage
    assert process_single_field(0.898, mapping(scale=100)) == pytest.approx(89.8)


def test_scale_of_zero_still_publishes():
    # Regression guard: a 0 value must not be mistaken for "filtered out"
    assert process_single_field(0.0, mapping(scale=100)) == 0.0


def test_filter_sees_the_raw_value_not_the_scaled_one():
    # The filter is written against the ROS units, so adding a scale must not
    # change which values it lets through
    m = mapping(scale=100, filter='lambda x: x < 1')
    assert process_single_field(0.5, m) == pytest.approx(50.0)


def test_filter_rejecting_returns_none_without_scaling():
    m = mapping(scale=100, filter='lambda x: x > 0.9')
    assert process_single_field(0.5, m) is None


def test_filter_alone_is_unchanged():
    assert process_single_field(5, mapping(filter='lambda x: x > 3')) == 5
    assert process_single_field(1, mapping(filter='lambda x: x > 3')) is None


def test_scale_refuses_strings_instead_of_repeating_them():
    # 'ab' * 3 == 'ababab' would silently publish nonsense
    assert process_single_field('ab', mapping(scale=3)) is None


def test_scale_refuses_bools_instead_of_treating_them_as_ints():
    # True * 100 == 100 would silently publish nonsense
    assert process_single_field(True, mapping(scale=100)) is None
