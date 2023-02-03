from plansys2_msgs import msg

import unified_planning as up
from unified_planning.io import PDDLReader
from unified_planning.model.operators import OperatorKind
from unified_planning.exceptions import UPValueError

from plansys2_upf_domain_expert import DomainExpert

from typing import List, Dict
import tempfile
import os
import re
from fractions import Fraction


class ProblemExpert():

    def __init__(self, domain_filename: str):
        # self.domain = PDDLReader().parse_problem(domain_filename, None)
        self.problem = None
        # self.goal = None
        # self.instances = None
        # self.predicates = None
        # self.functions = None
        self.domain_expert = DomainExpert(domain_filename)

        self.domain_pddl = None 
        self.problem_pddl = None
        with open(domain_filename) as f:
            ll = f.readlines()
            self.domain_pddl = ''.join(ll)
        
    def parseProblem(self, problem_str):
        domain_file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        problem_file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        try:
            domain_file.write(self.domain_pddl)
            problem_file.write(problem_str)
            domain_file.close()
            problem_file.close()

            self.problem = PDDLReader().parse_problem(domain_file.name, problem_file.name)
            self.problem_pddl = problem_str
            return self.problem
        finally:
            os.unlink(domain_file.name)
            os.unlink(problem_file.name)
        
        return None

    # TODO: this function replace the previous problem if present
    def addProblem(self, problem_str):
        # TODO: check if empty string
        problem = self.parseProblem(problem_str)
        return problem is not None

    # TODO: cpp code sobstitute goal
    def addProblemGoal(self, tree: msg.Tree):
        if self.problem is None:
            return False

        nodes = dict([(node.node_id, node) for node in tree.nodes])
        for node in tree.nodes:
            self.problem.add_goal(self.domain_expert.constructFNode(nodes, node))
        return True

    # TODO: check if instance already exists 
    def addProblemInstance(self, instance: msg.Param):
        if self.problem is None:
            return False
        
        try:
            type = self.problem.user_type(instance.type)
        except UPValueError:
            return False

        self.problem.add_object(up.model.Object(instance.name, type, self.problem.env))
        return True

    # TODO: check if predicate already exists, if it is valid
    # TODO: manage cases where types are not defined
    def addProblemPredicate(self, node: msg.Node):
        if self.problem is None:
            return False

        if self.existProblemPredicate(node):
            return False

        parameters = list()
        for p in node.parameters:
            try:
                type = self.problem.user_type(p.type)
            except UPValueError:
                return False
            parameters.append(up.model.Object(p.name, type, self.problem.env))

        fluent = up.model.Fluent(node.name, self.problem.env.type_manager.BoolType(), parameters, self.problem.env)
        fnode = self.problem.env.expression_manager.FluentExp(fluent, fluent.signature)
        self.problem.set_initial_value(fnode, self.problem.env.expression_manager.TRUE())
        # TODO: use add_fluent()?

        return True

    def addProblemFunction(self, node):
        if self.problem is None:
            return False

        if self.existProblemFunction(node):
            return False # TODO: update function instead of return false

        parameters = list()
        for p in node.parameters:
            try:
                type = self.problem.user_type(p.type)
            except UPValueError:
                return False
            parameters.append(up.model.Object(p.name, type, self.problem.env))
        
        print(f"parameters: {parameters}")
        fluent = up.model.Fluent(node.name, self.problem.env.type_manager.RealType(), parameters, self.problem.env)
        fnode = self.problem.env.expression_manager.FluentExp(fluent, fluent.signature)
        self.problem.set_initial_value(fnode, self.problem.env.expression_manager.Real(Fraction(0))) # TODO

        return True

    def getProblemGoal(self):
        if self.problem is None:
            return []
        
        # TODO: consider timed goals
        tree = msg.Tree()
        tree.nodes = list()
        and_node = msg.Node()
        and_node.node_type = self.domain_expert.map_types[OperatorKind.AND]
        and_node.node_id = len(tree.nodes)
        and_node.children = list()
        tree.nodes.append(and_node)

        for goal in self.problem.goals:
            node = self.domain_expert.constructTree(goal, tree.nodes)
            and_node.children.append(node.node_id)

        return tree

    def getProblemInstance(self, instance: str):
        if self.problem is None:
            return None

        for obj in self.problem.all_objects:
            if obj.name.lower() == instance.strip().lower():
                return self.domain_expert.constructParameters([obj])[0]
        return None

    def getProblemInstances(self):
        if self.problem is None:
            return []
        
        return self.domain_expert.constructParameters(self.problem.all_objects)

    def getProblemPredicate(self, predicate_str: str):
        if self.problem is None:
            return None
            
        match = re.match(r"^\s*\(\s*([\w-]+)([\s\w-]*)\)\s*$", predicate_str)
        if match is None:
            return None

        predicate_name = match.group(1)
        params_name = [arg.strip() for arg in match.group(2).split()]

        for i, fnode in enumerate(self.problem.explicit_initial_values):
            assert(fnode.is_fluent_exp())
            predicate = fnode.fluent()
            if predicate.type.is_bool_type() and predicate.name == predicate_name:
                predicate_msg = msg.Node()
                predicate_msg.node_type = msg.Node.PREDICATE
                predicate_msg.node_id = i
                predicate_msg.name = predicate.name
                params_map = dict([(p.name, fnode.args[j].object().name) for j,p in enumerate(predicate.signature)])
                predicate_msg.parameters = self.domain_expert.constructParameters(predicate.signature, params_map) # TODO: avoid construct parameters all the times
                
                params_match = (len(predicate_msg.parameters) == len(params_name))
                if not params_match:
                    continue
                for j, param in enumerate(predicate_msg.parameters):
                    if param.name.lower() != params_name[j].lower():
                        params_match = False
                        break
                if params_match:
                    return predicate_msg

        return None

    def getProblemPredicates(self):
        if self.problem is None:
            return []

        predicates = list()
        for i, fnode in enumerate(self.problem.explicit_initial_values):
            assert(fnode.is_fluent_exp())
            predicate = fnode.fluent()
            if predicate.type.is_bool_type():
                predicate_msg = msg.Node()
                predicate_msg.node_type = msg.Node.PREDICATE
                predicate_msg.node_id = i
                predicate_msg.name = predicate.name
                params_map = dict([(p.name, fnode.args[j].object().name) for j,p in enumerate(predicate.signature)])
                predicate_msg.parameters = self.domain_expert.constructParameters(predicate.signature, params_map)
                predicates.append(predicate_msg)
        return predicates

    def getProblemFunction(self, function_str: str): # TODO
        if self.problem is None:
            return None

        match = re.match(r"^\s*\(\s*([\w-]+)([\s\w-]*)\)\s*$", function_str)
        if match is None:
            return None

        function_name = match.group(1)
        params_name = [arg.strip() for arg in match.group(2).split()]

        for i, fnode in enumerate(self.problem.explicit_initial_values):
            assert(fnode.is_fluent_exp())
            function = fnode.fluent()
            if function.type.is_real_type() and function.name == function_name:
                function_msg = msg.Node()
                function_msg.node_type = msg.Node.FUNCTION
                function_msg.node_id = i
                function_msg.name = function.name
                params_map = dict([(p.name, fnode.args[j].object().name) for j,p in enumerate(function.signature)])
                function_msg.parameters = self.domain_expert.constructParameters(function.signature, params_map)
                function_msg.value = float(self.problem.explicit_initial_values[fnode].constant_value())

                params_match = (len(function_msg.parameters) == len(params_name))
                if not params_match:
                    continue
                for j, param in enumerate(function_msg.parameters):
                    if param.name.lower() != params_name[j].lower():
                        params_match = False
                        break
                if params_match:
                    return function_msg

        return None

    def getProblemFunctions(self): # TODO
        if self.problem is None:
            return []

        functions = list()
        for i, fnode in enumerate(self.problem.explicit_initial_values):
            assert(fnode.is_fluent_exp())
            function = fnode.fluent()
            if function.type.is_real_type():
                function_msg = msg.Node()
                function_msg.node_type = msg.Node.FUNCTION
                function_msg.node_id = i
                function_msg.name = function.name
                params_map = dict([(p.name, fnode.args[j].object().name) for j,p in enumerate(function.signature)])
                function_msg.parameters = self.domain_expert.constructParameters(function.signature, params_map)
                function_msg.value = float(self.problem.explicit_initial_values[fnode].constant_value())
                functions.append(function_msg)
        return functions

    def getProblem(self) -> str:
        if self.problem is None:
            return ""

        return self.problem_pddl

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
        # params type not considered
        params_str = ' '.join([p.name for p in node.parameters])
        predicate_str = f"({node.name} {params_str})"
        return self.getProblemPredicate(predicate_str) is not None

    def existProblemFunction(self, node: msg.Node):
        # params type not considered
        params_str = ' '.join([p.name for p in node.parameters])
        function_str = f"({node.name} {params_str})"
        return self.getProblemFunction(function_str) is not None

    def updateProblemFunction(self):
        pass

    def isProblemGoalSatisfied(self):
        pass

