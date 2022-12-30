from plansys2_msgs import srv
from plansys2_upf_problem_expert import ProblemExpert
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State

import rclpy
from rclpy.node import Node


class ProblemExpertNode(Node):

    def __init__(self):
        super().__init__('problem_expert')

        self.problem_expert = None
        self.state = State.PRIMARY_STATE_UNCONFIGURED
        self.declare_parameter("model_file", "")

        self.create_service(GetState, 'problem_expert/get_state',
                            self.get_state_service_callback)
        self.create_service(ChangeState, 'problem_expert/change_state',
                            self.change_state_service_callback)

        self.create_service(srv.AddProblem, 'problem_expert/add_problem',
                            self.add_problem_service_callback)
        self.create_service(srv.AddProblemGoal, 'problem_expert/add_problem_goal',
                            self.add_problem_goal_service_callback)
        self.create_service(srv.AffectParam, 'problem_expert/add_problem_instance',
                            self.add_problem_instance_service_callback)
        self.create_service(srv.AffectNode, 'problem_expert/add_problem_predicate',
                            self.add_problem_predicate_service_callback)
        self.create_service(srv.AffectNode, 'problem_expert/add_problem_function',
                            self.add_problem_function_service_callback)
        self.create_service(srv.GetProblemGoal, 'problem_expert/get_problem_goal',
                            self.get_problem_goal_service_callback)
        self.create_service(srv.GetProblemInstanceDetails, 'problem_expert/get_problem_instance', 
                            self.get_problem_instance_service_callback)
        self.create_service(srv.GetProblemInstances, 'problem_expert/get_problem_instances',
                            self.get_problem_instances_service_callback)
        self.create_service(srv.GetNodeDetails, 'problem_expert/get_problem_predicate',
                            self.get_problem_predicate_service_callback)
        self.create_service(srv.GetStates, 'problem_expert/get_problem_predicates',
                            self.get_problem_predicates_service_callback)
        self.create_service(srv.GetNodeDetails, 'problem_expert/get_problem_function',
                            self.get_problem_function_service_callback)
        self.create_service(srv.GetStates, 'problem_expert/get_problem_functions',
                            self.get_problem_functions_service_callback)
        self.create_service(srv.GetProblem, 'problem_expert/get_problem', 
                            self.get_problem_service_callback)
        self.create_service(srv.RemoveProblemGoal, 'problem_expert/remove_problem_goal',
                            self.remove_problem_goal_service_callback)
        self.create_service(srv.ClearProblemKnowledge, 'problem_expert/clear_problem_knowledge',
                            self.clear_problem_knowledge_service_callback)
        self.create_service(srv.AffectParam, 'problem_expert/remove_problem_instance',
                            self.remove_problem_instance_service_callback)
        self.create_service(srv.AffectNode, 'problem_expert/remove_problem_predicate',
                            self.remove_problem_predicate_service_callback)
        self.create_service(srv.AffectNode, 'problem_expert/remove_problem_function',
                            self.remove_problem_function_service_callback)
        self.create_service(srv.ExistNode, 'problem_expert/exist_problem_predicate',
                            self.exist_problem_predicate_service_callback)
        self.create_service(srv.ExistNode, 'problem_expert/exist_problem_function',
                            self.exist_problem_function_service_callback)
        self.create_service(srv.AffectNode, 'problem_expert/update_problem_function',
                            self.update_problem_function_service_callback)
        self.create_service(srv.IsProblemGoalSatisfied, 'problem_expert/is_problem_goal_satisfied',
                            self.is_problem_goal_satisfied_service_callback)

    def get_state_service_callback(self, request, response):
        response.current_state = State()
        response.current_state.id = self.state
        # response.current_state.label = "" # TODO
        return response

    def change_state_service_callback(self, request, response):
        transition_callback = {
            Transition.TRANSITION_CONFIGURE: self.on_configure,
            Transition.TRANSITION_ACTIVATE: self.on_activate,
            Transition.TRANSITION_DEACTIVATE: self.on_deactivate,
            Transition.TRANSITION_CLEANUP: self.on_cleanup,
            Transition.TRANSITION_ACTIVE_SHUTDOWN: self.on_shutdown,
            Transition.TRANSITION_CALLBACK_ERROR: self.on_error,
            Transition.TRANSITION_ON_ACTIVATE_ERROR: self.on_error,
            Transition.TRANSITION_ON_CLEANUP_ERROR: self.on_error,
            Transition.TRANSITION_ON_CONFIGURE_ERROR: self.on_error,
            Transition.TRANSITION_ON_ERROR_ERROR: self.on_error,
            Transition.TRANSITION_ON_DEACTIVATE_ERROR: self.on_error,
            Transition.TRANSITION_ON_SHUTDOWN_ERROR: self.on_error,
        }
        if request.transition.id in transition_callback:
            response.success = transition_callback[request.transition.id]()
        else:
            response.success = False

        return response

    def on_configure(self):
        self.get_logger().info(f"[{self.get_name()}] Configuring...")
        model_file = self.get_parameter(
            "model_file").get_parameter_value().string_value
        try:
            self.problem_expert = ProblemExpert(model_file)
            self.state = State.PRIMARY_STATE_INACTIVE
        except:
            self.get_logger().error("PDDL syntax error")
            return False

        self.get_logger().info(f"[{self.get_name()}] Configured")
        return True

    def on_activate(self):
        self.get_logger().info(f"[{self.get_name()}] Activating...")
        self.state = State.PRIMARY_STATE_ACTIVE
        self.get_logger().info(f"[{self.get_name()}] Activated")
        return True

    def on_deactivate(self):
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")
        self.state = State.PRIMARY_STATE_INACTIVE
        self.get_logger().info(f"[{self.get_name()}] Deactivated")
        return True

    def on_cleanup(self):
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")
        return True

    def on_shutdown(self):
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")
        self.state = State.PRIMARY_STATE_FINALIZED
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        return True

    def on_error(self):
        self.get_logger().error(f"[{self.get_name()}] Error transition")
        return True


    def add_problem_service_callback(self, request, response):
        self.get_logger().info(f'add_problem::Incoming request: {request}')

        if self.problem_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.success = self.problem_expert.addProblem(request.problem)

        return response

    def add_problem_goal_service_callback(self, request, response):
        self.get_logger().info(f'add_problem_goal::Incoming request: {request}')

    def add_problem_instance_service_callback(self, request, response):
        self.get_logger().info(f'add_problem_instance::Incoming request: {request}')

    def add_problem_predicate_service_callback(self, request, response):
        self.get_logger().info(f'add_problem_predicate::Incoming request: {request}')

    def add_problem_function_service_callback(self, request, response):
        self.get_logger().info(f'add_problem_function::Incoming request: {request}')

    def get_problem_goal_service_callback(self, request, response):
        self.get_logger().info(f'get_problem_goal::Incoming request: {request}')

        if self.problem_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.tree = self.problem_expert.getProblemGoal()
            response.success = True
        return response

    def get_problem_instance_service_callback(self, request, response):
        self.get_logger().info(f'get_problem_instance::Incoming request: {request}')

        if self.problem_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            instance = self.problem_expert.getProblemInstance(request.instance)
            if instance is None:
                response.error_info = "Instance not found"
                response.success = False
            else:
                response.instance = instance
                response.success = True
        return response

    def get_problem_instances_service_callback(self, request, response):
        self.get_logger().info(f'get_problem_instances::Incoming request: {request}')

        if self.problem_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.instances = self.problem_expert.getProblemInstances()
            response.success = True
        return response

    def get_problem_predicate_service_callback(self, request, response):
        self.get_logger().info(f'get_problem_predicate::Incoming request: {request}')

        if self.problem_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            predicate_msg = self.problem_expert.getProblemPredicate(request.expression)
            if predicate_msg is None:
                response.error_info = "Predicate not found"
                response.success = False
            else:
                response.node = predicate_msg
                response.success = True
        return response

    def get_problem_predicates_service_callback(self, request, response):
        self.get_logger().info(f'get_problem_predicates::Incoming request: {request}')

        if self.problem_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.states = self.problem_expert.getProblemPredicates()
            response.success = True
        return response

    def get_problem_function_service_callback(self, request, response):
        self.get_logger().info(f'get_problem_function::Incoming request: {request}')

    def get_problem_functions_service_callback(self, request, response):
        self.get_logger().info(f'get_problem_functions::Incoming request: {request}')

    def get_problem_service_callback(self, request, response):
        self.get_logger().info(f'get_problem::Incoming request: {request}')

        if self.problem_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.problem = self.problem_expert.getProblem()
            response.success = True
        return response

    def remove_problem_goal_service_callback(self, request, response):
        self.get_logger().info(f'remove_problem_goal::Incoming request: {request}')

    def clear_problem_knowledge_service_callback(self, request, response):
        self.get_logger().info(f'clear_problem_knowledge::Incoming request: {request}')

    def remove_problem_instance_service_callback(self, request, response):
        self.get_logger().info(f'remove_problem_instance::Incoming request: {request}')

    def remove_problem_predicate_service_callback(self, request, response):
        self.get_logger().info(f'remove_problem_predicate::Incoming request: {request}')

    def remove_problem_function_service_callback(self, request, response):
        self.get_logger().info(f'remove_problem_function::Incoming request: {request}')

    def exist_problem_predicate_service_callback(self, request, response):
        self.get_logger().info(f'exist_problem_predicate::Incoming request: {request}')

    def exist_problem_function_service_callback(self, request, response):
        self.get_logger().info(f'exist_problem_function::Incoming request: {request}')

    def update_problem_function_service_callback(self, request, response):
        self.get_logger().info(f'update_problem_function::Incoming request: {request}')

    def is_problem_goal_satisfied_service_callback(self, request, response):
        self.get_logger().info(f'is_problem_goal_satisfied::Incoming request: {request}')


def main():
    rclpy.init()

    problem_expert_node = ProblemExpertNode()

    rclpy.spin(problem_expert_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
