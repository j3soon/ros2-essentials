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

from typing import List, Text, Union

from launch.condition import Condition
from launch.conditions import evaluate_condition_expression
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions


class AndCondition(Condition):
    """
    Encapsulates an and condition to be evaluated when launching.

    This condition takes a list or tuple of string expressions that are lexically evaluated as a
    boolean, but the expressions may consist of launch.Substitution or launch.Condition instances.

    This condition will raise a TypeError if a list or tuple is not provided.
    """

    def __init__(
        self,
        predicate_expressions: List[Union[SomeSubstitutionsType, Condition]]
    ) -> None:
        if isinstance(predicate_expressions, str):
            raise TypeError(
                'interbotix_common_modules.launch.AndCondition given a single expression. Two or '
                'more expressions are expected.'
            )
        self.__conditions = []
        self.__substitutions = []
        to_be_normalized = []
        # Separate out conditions to be evaluated on their own
        for pe in predicate_expressions:
            if isinstance(pe, Condition):
                self.__conditions.append(pe)
            else:
                to_be_normalized.append(pe)
        self.__substitutions = [
            normalize_to_list_of_substitutions(condition) for condition in to_be_normalized
        ]
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        return all(
            [evaluate_condition_expression(context, sub) for sub in self.__substitutions]
        ) and all([condition.evaluate(context) for condition in self.__conditions])

    def describe(self) -> Text:
        return self.__repr__()
