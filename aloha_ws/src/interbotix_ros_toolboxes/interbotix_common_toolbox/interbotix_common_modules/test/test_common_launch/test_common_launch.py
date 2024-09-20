# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from interbotix_common_modules.launch import AndCondition, OrCondition
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution

import pytest


class MockLaunchContext:

    def perform_substitution(self, substitution):
        return substitution.perform(self)


def test_and_condition():
    """Test AndCondition class."""
    class MockLaunchContext:

        def perform_substitution(self, substitution):
            return substitution.perform(self)

    lc = MockLaunchContext()
    test_cases = [
        (['true', 'True'], True),
        (('true', 'True'), True),
        (['true', 'True', '1'], True),
        (['true', '1'], True),
        (['false', 'true'], False),
        (['true', 'false'], False),
        (['true', '0'], False),
        (['true', IfCondition('true')], True),
    ]

    for conditions, expected in test_cases:
        assert AndCondition(conditions).evaluate(lc) is expected

    with pytest.raises(TypeError):
        AndCondition('true').evaluate(lc)


def test_or_condition():
    """Test OrCondition class."""
    lc = MockLaunchContext()
    test_cases = [
        (['true', 'True', 'TRUE'], True),
        (('true', 'True', 'TRUE'), True),
        (['false', 'False', 'FALSE'], False),
        (['true', 'false'], True),
        (['true', IfCondition('false')], True),
    ]

    for conditions, expected in test_cases:
        assert OrCondition(conditions).evaluate(lc) is expected

    with pytest.raises(TypeError):
        OrCondition(TextSubstitution(text='true')).evaluate(lc)
