from plansys2_msgs import srv, msg
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.node import Node

from typing import List


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')


        self.add_problem_client = self.create_client(srv.AddProblem, 'problem_expert_upf/add_problem')
        self.add_problem_goal_client = self.create_client(srv.AddProblemGoal, 'problem_expert_upf/add_problem_goal')
        self.add_problem_instance_client = self.create_client(srv.AffectParam, 'problem_expert_upf/add_problem_instance')
        self.add_problem_predicate_client = self.create_client(srv.AffectNode, 'problem_expert_upf/add_problem_predicate')
        self.add_problem_function_client = self.create_client(srv.AffectNode, 'problem_expert_upf/add_problem_function')
        self.get_problem_goal_client = self.create_client(srv.GetProblemGoal, 'problem_expert_upf/get_problem_goal')
        self.get_problem_instance_details_client = self.create_client(srv.GetProblemInstanceDetails, 'problem_expert_upf/get_problem_instance')
        self.get_problem_instances_client = self.create_client(srv.GetProblemInstances, 'problem_expert_upf/get_problem_instances')
        self.get_problem_predicate_details_client = self.create_client(srv.GetNodeDetails, 'problem_expert_upf/get_problem_predicate')
        self.get_problem_predicates_client = self.create_client(srv.GetStates, 'problem_expert_upf/get_problem_predicates')
        self.get_problem_function_details_client = self.create_client(srv.GetNodeDetails, 'problem_expert_upf/get_problem_function')
        self.get_problem_functions_client = self.create_client(srv.GetStates, 'problem_expert_upf/get_problem_functions')
        self.get_problem_client = self.create_client(srv.GetProblem, 'problem_expert_upf/get_problem')
        self.remove_problem_goal_client = self.create_client(srv.RemoveProblemGoal, 'problem_expert_upf/remove_problem_goal')
        self.clear_problem_knowledge_client = self.create_client(srv.ClearProblemKnowledge, 'problem_expert_upf/clear_problem_knowledge')
        self.remove_problem_instance_client = self.create_client(srv.AffectParam, 'problem_expert_upf/remove_problem_instance')
        self.remove_problem_predicate_client = self.create_client(srv.AffectNode, 'problem_expert_upf/remove_problem_predicate')
        self.remove_problem_function_client = self.create_client(srv.AffectNode, 'problem_expert_upf/remove_problem_function')
        self.exist_problem_predicate_client = self.create_client(srv.ExistNode, 'problem_expert_upf/exist_problem_predicate')
        self.exist_problem_function_client = self.create_client(srv.ExistNode, 'problem_expert_upf/exist_problem_function')
        self.update_problem_function_client = self.create_client(srv.AffectNode, 'problem_expert_upf/update_problem_function')
        self.is_problem_goal_satisfied_client = self.create_client(srv.IsProblemGoalSatisfied, 'problem_expert_upf/is_problem_goal_satisfied')


        self.add_problem_client_cpp = self.create_client(srv.AddProblem, 'problem_expert/add_problem')
        self.add_problem_goal_client_cpp = self.create_client(srv.AddProblemGoal, 'problem_expert/add_problem_goal')
        self.add_problem_instance_client_cpp = self.create_client(srv.AffectParam, 'problem_expert/add_problem_instance')
        self.add_problem_predicate_client_cpp = self.create_client(srv.AffectNode, 'problem_expert/add_problem_predicate')
        self.add_problem_function_client_cpp = self.create_client(srv.AffectNode, 'problem_expert/add_problem_function')
        self.get_problem_goal_client_cpp = self.create_client(srv.GetProblemGoal, 'problem_expert/get_problem_goal')
        self.get_problem_instance_details_client_cpp = self.create_client(srv.GetProblemInstanceDetails, 'problem_expert/get_problem_instance')
        self.get_problem_instances_client_cpp = self.create_client(srv.GetProblemInstances, 'problem_expert/get_problem_instances')
        self.get_problem_predicate_details_client_cpp = self.create_client(srv.GetNodeDetails, 'problem_expert/get_problem_predicate')
        self.get_problem_predicates_client_cpp = self.create_client(srv.GetStates, 'problem_expert/get_problem_predicates')
        self.get_problem_function_details_client_cpp = self.create_client(srv.GetNodeDetails, 'problem_expert/get_problem_function')
        self.get_problem_functions_client_cpp = self.create_client(srv.GetStates, 'problem_expert/get_problem_functions')
        self.get_problem_client_cpp = self.create_client(srv.GetProblem, 'problem_expert/get_problem')
        self.remove_problem_goal_client_cpp = self.create_client(srv.RemoveProblemGoal, 'problem_expert/remove_problem_goal')
        self.clear_problem_knowledge_client_cpp = self.create_client(srv.ClearProblemKnowledge, 'problem_expert/clear_problem_knowledge')
        self.remove_problem_instance_client_cpp = self.create_client(srv.AffectParam, 'problem_expert/remove_problem_instance')
        self.remove_problem_predicate_client_cpp = self.create_client(srv.AffectNode, 'problem_expert/remove_problem_predicate')
        self.remove_problem_function_client_cpp = self.create_client(srv.AffectNode, 'problem_expert/remove_problem_function')
        self.exist_problem_predicate_client_cpp = self.create_client(srv.ExistNode, 'problem_expert/exist_problem_predicate')
        self.exist_problem_function_client_cpp = self.create_client(srv.ExistNode, 'problem_expert/exist_problem_function')
        self.update_problem_function_client_cpp = self.create_client(srv.AffectNode, 'problem_expert/update_problem_function')
        self.is_problem_goal_satisfied_client_cpp = self.create_client(srv.IsProblemGoalSatisfied, 'problem_expert/is_problem_goal_satisfied')


        self.de_get_state_service = self.create_client(
            GetState, 'problem_expert_upf/get_state')
        self.de_change_state_service = self.create_client(
            ChangeState, 'problem_expert_upf/change_state')
        self.de_get_state_service_cpp = self.create_client(
            GetState, 'problem_expert/get_state')
        self.de_change_state_service_cpp = self.create_client(
            ChangeState, 'problem_expert/change_state')

        self.getState()
        self.changeState(Transition.TRANSITION_CONFIGURE)
        self.getState()
        self.changeState(Transition.TRANSITION_ACTIVATE)
        self.getState()

    def getState(self):
        request = GetState.Request()

        while not self.de_get_state_service_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.de_get_state_service_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f"response: {response}")

        while not self.de_get_state_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')

        future = self.de_get_state_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f"response2: {response}")

    def changeState(self, transition):
        request = ChangeState.Request()
        request.transition.id = transition

        while not self.de_change_state_service_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.de_change_state_service_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f"response: {response}")

        while not self.de_change_state_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')

        future = self.de_change_state_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f"response2: {response}")


    def addProblem(self, problem_str: str):
        request = srv.AddProblem.Request()
        request.problem = problem_str

        while not self.add_problem_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.add_problem_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.add_problem_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.add_problem_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error addProblem")
        else:
            print(response)

        return response

    def addProblemGoal(self, node):
        request = srv.AffectParam.Request()
        request.param = msg.Param()
        request.tree = msg.Tree()
        request.tree.nodes = [node]

        while not self.add_problem_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.add_problem_goal_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.add_problem_goal_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.add_problem_goal_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error addProblemGoal")
        else:
            print(response)

        return response

    def addProblemInstance(self, name: str, type: str):
        request = srv.AffectParam.Request()
        request.param = msg.Param()
        request.param.name = name
        request.param.type = type

        while not self.add_problem_instance_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.add_problem_instance_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.add_problem_instance_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.add_problem_instance_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error addProblemInstance")
        else:
            print(response)

        return response

    def addProblemPredicate(self):
        pass

    def addProblemFunction(self):
        pass

    def getProblemGoal(self):
        request = srv.GetProblemGoal.Request()

        while not self.get_problem_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_goal_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.get_problem_goal_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_goal_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error getProblemGoal")
        else:
            print(response)

        return response


    def getProblemInstance(self, instance):
        request = srv.GetProblemInstanceDetails.Request()
        request.instance = instance

        while not self.get_problem_instance_details_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_instance_details_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.get_problem_instance_details_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_instance_details_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error getProblemInstance")
        else:
            print(response)

        return response

    def getProblemInstances(self):
        request = srv.GetProblemInstances.Request()

        while not self.get_problem_instances_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_instances_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_problem_instances_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_instances_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error getProblemInstances")
        else:
            print(response)

        return response

    def getProblemPredicate(self, predicate: str):
        request = srv.GetNodeDetails.Request()
        request.expression = predicate

        while not self.get_problem_predicate_details_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_predicate_details_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.get_problem_predicate_details_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_predicate_details_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error getProblemPredicate")
        else:
            print(response)

        return response

    def getProblemPredicates(self):
        request = srv.GetStates.Request()

        while not self.get_problem_predicates_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_predicates_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.get_problem_predicates_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_predicates_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error getProblemPredicates")
        else:
            print(response)

        return response

    def getProblemFunction(self, function: str):
        request = srv.GetNodeDetails.Request()
        request.expression = function

        while not self.get_problem_function_details_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_function_details_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.get_problem_function_details_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_function_details_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error getProblemFunction")
        else:
            print(response)

        return response

    def getProblemFunctions(self):
        request = srv.GetStates.Request()

        while not self.get_problem_functions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_functions_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.get_problem_functions_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_functions_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error getProblemFunctions")
        else:
            print(response)

        return response


    def getProblem(self):
        request = srv.GetProblem.Request()

        while not self.get_problem_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.get_problem_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.get_problem_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response.problem != response_cpp.problem:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error getProblem")
        else:
            print(response)

        return response

    def removeProblemGoal(self):
        pass

    def clearProblemKnowledge(self):
        pass

    def removeProblemInstance(self):
        pass

    def removeProblemPredicate(self):
        pass

    def removeProblemFunction(self):
        pass

    def existProblemPredicate(self, node: msg.Node):
        request = srv.ExistNode.Request()
        request.node = node

        while not self.exist_problem_predicate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.exist_problem_predicate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.exist_problem_predicate_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.exist_problem_predicate_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error existProblemPredicate")
        else:
            print(response)

        return response

    def existProblemFunction(self, node: msg.Node):
        request = srv.ExistNode.Request()
        request.node = node

        while not self.exist_problem_function_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.exist_problem_function_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        while not self.exist_problem_function_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.exist_problem_function_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response != response_cpp:
            print("========================================")
            print(response)
            print("========================================")
            print(response_cpp)
            print("========================================")
            print("error existProblemFunction")
        else:
            print(response)

        return response

    def updateProblemFunction(self):
        pass

    def isProblemGoalSatisfied(self):
        pass



def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()

    with open("planning/plansys2_ws/src/plansys2_upf_problem_expert/tmp/problem_simple_1.pddl") as f:
        problem_str = ''.join(f.readlines())

    minimal_client.addProblem(problem_str)
    minimal_client.getProblemGoal()
    # minimal_client.getProblem()
    # minimal_client.getProblemPredicates()
    # minimal_client.getProblemFunctions()
    # minimal_client.getProblemFunction("(room_distance kitchen bedroom)")
    # minimal_client.getProblemPredicate("(robot_at leia kitchen)")
    # minimal_client.getProblemInstances()
    # minimal_client.getProblemInstance("jack")

    # node = msg.Node()
    # node.node_type = msg.Node.PREDICATE
    # node.name = "robot_at"
    # node.parameters.append(msg.Param())
    # node.parameters[-1].name = "leia"
    # node.parameters.append(msg.Param())
    # node.parameters[-1].name = "kitchen"
    # minimal_client.existProblemPredicate(node)

    # node = msg.Node()
    # node.node_type = msg.Node.FUNCTION
    # node.name = "room_distance"
    # node.parameters.append(msg.Param())
    # node.parameters[-1].name = "kitchen"
    # node.parameters.append(msg.Param())
    # node.parameters[-1].name = "bedroom"
    # minimal_client.existProblemFunction(node)

    # minimal_client.addProblemInstance("my_object", "robot")
    # minimal_client.getProblemInstances()

    node = msg.Node()
    node.node_type = msg.Node.PREDICATE
    node.name = "robot_at"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "leia"
    node.parameters[-1].type = "person"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "kitchen"
    node.parameters[-1].type = "room"
    minimal_client.addProblemGoal(node)
    minimal_client.getProblemGoal()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
