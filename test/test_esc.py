# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the ESC arming, direction-change and dead-band logic."""

import pytest
from ros2_pca9685.esc import EscConfig, EscSequencer, FORWARD, REVERSE, sequence_duration


def test_default_config_is_a_plain_pass_through():
    config = EscConfig()
    assert config.configured is False
    assert config.shape(0.0, -1.0, 1.0) == 0.0
    assert config.shape(0.37, -1.0, 1.0) == pytest.approx(0.37)
    assert config.shape(-0.8, -1.0, 1.0) == pytest.approx(-0.8)
    assert config.shape(2.0, -1.0, 0.5) == pytest.approx(0.5)


def test_shape_spreads_commands_over_the_usable_band():
    config = EscConfig(deadband=0.05, forward_start=0.3, reverse_start=0.1)
    assert config.configured is True
    assert config.shape(0.05, -1.0, 1.0) == 0.0
    assert config.shape(-0.04, -1.0, 1.0) == 0.0
    assert config.shape(0.0500001, -1.0, 1.0) == pytest.approx(0.3, abs=1e-6)
    assert config.shape(1.0, -1.0, 1.0) == pytest.approx(1.0)
    assert config.shape(0.525, -1.0, 1.0) == pytest.approx(0.65)
    assert config.shape(-1.0, -1.0, 1.0) == pytest.approx(-1.0)
    assert config.shape(-0.525, -1.0, 1.0) == pytest.approx(-0.55)


def test_shape_respects_asymmetric_limits():
    config = EscConfig(deadband=0.02, forward_start=0.2, reverse_start=0.05)
    assert config.shape(0.5, -0.3, 0.5) == pytest.approx(0.5)
    assert config.shape(0.9, -0.3, 0.5) == pytest.approx(0.5)
    assert config.shape(0.26, -0.3, 0.5) == pytest.approx(0.35)
    assert config.shape(-0.3, -0.3, 0.5) == pytest.approx(-0.3)
    assert config.shape(-0.16, -0.3, 0.5) == pytest.approx(-0.175)
    # A limit inside the dead-band disables that direction altogether.
    assert config.shape(0.5, -0.3, 0.0) == 0.0
    assert config.shape(-0.5, 0.0, 0.5) == 0.0


def test_sequence_duration():
    assert sequence_duration(()) == 0.0
    assert sequence_duration(((0.0, 2.5), (0.05, 0.5), (0.0, 0.5))) == pytest.approx(3.5)


def test_arming_runs_the_steps_then_hands_over():
    config = EscConfig(arming=((0.0, 2.0), (0.05, 0.5), (0.0, 0.5)))
    esc = EscSequencer(config, -1.0, 1.0)
    assert esc.output(0.0) == 0.0
    assert esc.busy is False
    assert esc.arm(10.0) is True
    assert esc.busy is True
    assert esc.sequence == 'arming'
    esc.set_wanted(0.8)  # commands during arming are held back
    assert esc.output(10.0) == 0.0
    assert esc.output(11.99) == 0.0
    assert esc.output(12.0) == 0.05
    assert esc.output(12.49) == 0.05
    assert esc.output(12.5) == 0.0
    assert esc.output(12.999) == 0.0
    assert esc.output(13.0) == pytest.approx(0.8)
    assert esc.busy is False
    assert esc.sequence == ''
    assert esc.mode == FORWARD


def test_arm_without_a_sequence_does_nothing():
    esc = EscSequencer(EscConfig(), -1.0, 1.0)
    assert esc.arm(0.0) is False
    assert esc.busy is False


def test_reverse_after_forward_runs_the_to_reverse_steps():
    config = EscConfig(to_reverse=((-0.3, 0.25), (0.0, 0.25)))
    esc = EscSequencer(config, -1.0, 1.0)
    esc.set_wanted(0.5)
    assert esc.output(0.0) == pytest.approx(0.5)
    assert esc.mode == FORWARD
    esc.set_wanted(-0.4)
    assert esc.output(1.0) == pytest.approx(-0.3)  # brake tap
    assert esc.sequence == 'to_reverse'
    assert esc.output(1.2) == pytest.approx(-0.3)
    assert esc.output(1.25) == 0.0  # neutral
    assert esc.output(1.5) == pytest.approx(-0.4)  # the reverse command itself
    assert esc.mode == REVERSE
    assert esc.busy is False
    # Staying in reverse needs no further sequence.
    esc.set_wanted(-0.6)
    assert esc.output(1.75) == pytest.approx(-0.6)
    # Forward again is direct when no to_forward sequence is configured.
    esc.set_wanted(0.2)
    assert esc.output(2.0) == pytest.approx(0.2)
    assert esc.mode == FORWARD


def test_forward_after_reverse_runs_the_to_forward_steps():
    config = EscConfig(to_forward=((0.0, 0.25),))
    esc = EscSequencer(config, -1.0, 1.0)
    esc.set_wanted(-0.5)
    assert esc.output(0.0) == pytest.approx(-0.5)  # first reverse is direct
    assert esc.mode == REVERSE
    esc.set_wanted(0.5)
    assert esc.output(1.0) == 0.0
    assert esc.sequence == 'to_forward'
    assert esc.output(1.25) == pytest.approx(0.5)
    assert esc.mode == FORWARD


def test_neutral_commands_keep_the_mode_and_need_no_sequence():
    config = EscConfig(to_reverse=((-0.3, 0.25),), deadband=0.05)
    esc = EscSequencer(config, -1.0, 1.0)
    esc.set_wanted(0.5)
    esc.output(0.0)
    esc.set_wanted(0.0)
    assert esc.output(1.0) == 0.0
    assert esc.mode == FORWARD
    esc.set_wanted(-0.03)  # inside the dead-band: still neutral, no sequence
    assert esc.output(2.0) == 0.0
    assert esc.busy is False
    esc.set_wanted(-0.5)
    assert esc.output(3.0) == pytest.approx(-0.3)
    assert esc.output(3.25) == pytest.approx(-(0.45 / 0.95))  # shaped for the dead-band


def test_changing_your_mind_during_a_sequence():
    config = EscConfig(to_reverse=((-0.3, 0.25), (0.0, 0.25)), to_forward=((0.0, 0.25),))
    esc = EscSequencer(config, -1.0, 1.0)
    esc.set_wanted(0.5)
    esc.output(0.0)
    esc.set_wanted(-0.5)
    assert esc.output(1.0) == pytest.approx(-0.3)
    esc.set_wanted(0.5)  # back to forward while the reverse sequence runs
    assert esc.output(1.3) == 0.0  # the sequence still completes
    assert esc.output(1.5) == 0.0  # then the to_forward sequence starts
    assert esc.sequence == 'to_forward'
    assert esc.output(1.75) == pytest.approx(0.5)
    assert esc.mode == FORWARD


def test_arming_then_reverse_chains_the_sequences():
    config = EscConfig(arming=((0.0, 1.0),), to_reverse=((-0.3, 0.25),))
    esc = EscSequencer(config, -1.0, 1.0)
    esc.arm(0.0)
    esc.set_wanted(-0.5)
    assert esc.output(0.5) == 0.0
    assert esc.output(1.0) == pytest.approx(-0.3)  # arming done, reverse entry starts
    assert esc.sequence == 'to_reverse'
    assert esc.output(1.25) == pytest.approx(-0.5)


def test_cancel_and_reconfigure():
    config = EscConfig(arming=((0.0, 1.0),), deadband=0.1)
    esc = EscSequencer(config, -1.0, 1.0)
    esc.arm(0.0)
    esc.cancel()
    assert esc.busy is False
    esc.set_wanted(0.5)
    assert esc.output(0.1) == pytest.approx(0.4 / 0.9)  # shaped for the dead-band
    esc.reconfigure(EscConfig(), -1.0, 0.5)
    assert esc.config == EscConfig()
    assert esc.output(0.2) == pytest.approx(0.5)
    assert esc.mode == FORWARD
    assert esc.wanted == 0.5
