from plansys2_msgs import msg

import unified_planning as up
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.model.operators import OperatorKind
from unified_planning.model import Fluent
from unified_planning.model.object import Object
from unified_planning.exceptions import UPValueError

from plansys2_upf_domain_expert import DomainExpert

from typing import List, Dict
import tempfile
import os
import re
from fractions import Fraction


class ProblemExpert():

    def __init__(self, domain_filename: str):
        self.problem = None
        self.domain_expert = DomainExpert(domain_filename)

    # TODO: manage exceptions
    def constructFNode(self, nodes, node: msg.Node) -> up.model.fnode.FNode:
        node_type = None
        args = None
        payload = None
        fnode = None

        if node.node_type == msg.Node.NUMBER:
            # node_type = self.inverse_map[node.node_type]
            fnode = self.domain.env.expression_manager.Real(Fraction(node.value))

        elif node.node_type in self.inverse_map:
            node_type = self.inverse_map[node.node_type]
            args = [self.constructFNode(nodes, nodes[child_id]) for child_id in node.children]

        elif node.node_type in (msg.Node.PREDICATE, msg.Node.FUNCTION):
            # node_type = OperatorKind.FLUENT_EXP
            # TODO: Object or Parameter?
            parameters = [Object(p.name, self.domain.user_type(p.type), self.domain.env) for p in node.parameters] # TODO: self.domain.user_type(p.type) can throw an execption
            if node.node_type == msg.Node.PREDICATE:
                fluent = Fluent(node.name, self.domain.env.type_manager.BoolType(), parameters, self.domain.env)
            else:
                fluent = Fluent(node.name, self.domain.env.type_manager.RealType(), parameters, self.domain.env)
            fnode = self.domain.env.expression_manager.FluentExp(fluent, fluent.signature)

        elif node.node_type == msg.Node.EXPRESSION:
            pass # TODO
        elif node.node_type == msg.Node.FUNCTION_MODIFIER:
            pass # TODO

        if fnode is None:
            fnode = self.domain.env.expression_manager.create_node(node_type, args, payload)

        return fnode

    # TODO: this function replaces the previous problem if present
    def addProblem(self, problem_str):
        # TODO: check if empty string
        try:
            self.problem = PDDLReader().parse_problem_string(self.domain_expert.getDomain(), problem_str)
            return True
        except:
            return False

    # this function sobstitutes the goal if present
    def addProblemGoal(self, tree: msg.Tree):
        if self.problem is None:
            return False

        self.removeProblemGoal()
        nodes = dict([(node.node_id, node) for node in tree.nodes])
        for node in tree.nodes:
            self.problem.add_goal(self.constructFNode(nodes, node))
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
            return None, None
            
        match = re.match(r"^\s*\(\s*([\w-]+)([\s\w-]*)\)\s*$", predicate_str)
        if match is None:
            return None, None

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
                    return fnode, predicate_msg

        return None, None

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

    def getProblemFunction(self, function_str: str):
        if self.problem is None:
            return None, None

        match = re.match(r"^\s*\(\s*([\w-]+)([\s\w-]*)\)\s*$", function_str)
        if match is None:
            return None, None

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
                    return fnode, function_msg

        return None, None

    def getProblemFunctions(self):
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

        return PDDLWriter(self.problem.clone()).get_problem() # TODO: clone should be avoided

    # TODO: remove useless and-node
    def removeProblemGoal(self):
        if self.problem is None:
            return False

        self.problem.clear_goals()
        return True

    # TODO
    def clearProblemKnowledge(self):
        pass

    # TODO: type not checked
    def removeProblemInstance(self, instance: msg.Param):
        if self.problem is None:
            return False
        
        for obj in self.problem.all_objects:
            if obj.name.lower() == instance.name.lower():
                self.problem._objects.remove(obj)
                return True
        return False

    def removeProblemPredicate(self, node: msg.Node):
        if self.problem is None:
            return False
        
        params_str = ' '.join([p.name for p in node.parameters])
        predicate_str = f"({node.name} {params_str})"
        predicate, predicate_msg = self.getProblemPredicate(predicate_str)
        if predicate is None:
            return False
        else:
            del self.problem._initial_value[predicate]
            return True

    def removeProblemFunction(self, node: msg.Node):
        if self.problem is None:
            return False
        
        params_str = ' '.join([p.name for p in node.parameters])
        function_str = f"({node.name} {params_str})"
        function, function_msg = self.getProblemFunction(function_str)
        if function is None:
            return False
        else:
            del self.problem._initial_value[function]
            return True

    def existProblemPredicate(self, node: msg.Node):
        if self.problem is None:
            return False

        # TODO: params type not considered
        params_str = ' '.join([p.name for p in node.parameters])
        predicate_str = f"({node.name} {params_str})"
        return self.getProblemPredicate(predicate_str)[0] is not None

    def existProblemFunction(self, node: msg.Node):
        if self.problem is None:
            return False

        # TODO: params type not considered
        params_str = ' '.join([p.name for p in node.parameters])
        function_str = f"({node.name} {params_str})"
        return self.getProblemFunction(function_str)[0] is not None

    # TODO
    def updateProblemFunction(self):
        pass

    # TODO
    def isProblemGoalSatisfied(self):
        pass
