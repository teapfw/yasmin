#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from threading import Thread

from yasmin import State
from yasmin import Blackboard
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, CONTINGENCY_FAILED

class ContingentActionState(State):
    """
    Represents an ActionState that may be cancelled by a parallel State.

    The contingency state's only admissable outcome is SUCCEED; any other
    outcome will result in a cancellation of the action state.

    The outcomes are the same as those of the ActionState, plus CONTINGENCY_FAILED
    in case of the contingency state's failure.

    Attributes:
        _node (Node): The ROS 2 node instance used to communicate with the action server.
        _action_state (ActionState): The YASMIN Action State
        _contingency_state (State): The YASMIN State monitoring the contingency requirement
    """

    def __init__(
        self,
        action_state: ActionState,
        contingency_state: State,
    ) -> None:
        """

        Parameters:
            action_state (ActionState): The ActionState to run
            contingency_state (State): The State upon which the action is contingent

        Raises:
            None
        """

        ## Store states
        self._action_state = action_state
        self._contingency_state = contingency_state

        ## Contingent acton outcome
        self._action_outcome = None

        _outcomes = [CONTINGENCY_FAILED] + self._action_state._outcomes

        super().__init__(_outcomes)

    def _execute_action(self, blackboard: Blackboard) -> None:
        self._action_outcome = self._action_state.execute(blackboard=blackboard)

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the action state by sending a goal to the action server.

        This method waits for the action server to be available, sends the goal,
        and waits for the action to complete, handling feedback and results.

        Parameters:
            blackboard (Blackboard): The blackboard instance used for state management.

        Returns:
            str: The outcome of the action execution (e.g., SUCCEED, ABORT, CANCEL, TIMEOUT).

        Raises:
            Exception: Raises an exception if any error occurs during action execution.
        """

        action_thread = Thread(
            target=self._execute_action,
            args=(blackboard,))
        action_thread.start()

        while self._contingency_state.execute(blackboard=blackboard) is SUCCEED:
            if self._action_outcome is not None:
                # Action completed; return appropriately
                action_thread.join()
                return self._action_outcome
        
        # Contingency failed; cancel the state
        self._action_state.cancel_state()
        action_thread.join()

        return CONTINGENCY_FAILED