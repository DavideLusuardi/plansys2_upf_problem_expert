from plansys2_msgs import msg
from plansys2_upf_domain_expert import DomainExpert

import unified_planning as up
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.model.operators import OperatorKind
from unified_planning.model import Fluent
from unified_planning.model.object import Object
from unified_planning.exceptions import UPValueError
from unified_planning.io.pddl_reader import CaseInsensitiveToken, nested_expr

import re
from fractions import Fraction

from typing import List, Dict, Optional

class ProblemExpert():

    def __init__(self, domain_filename: str):
        self.domain_expert = DomainExpert(domain_filename)
        self.problem = PDDLReader().parse_problem_string(self.domain_expert.getDomain())

        self.operators_map = {
            # msg.Node.UNKNOWN: '', # TODO
            msg.Node.AND: 'and',
            msg.Node.OR: 'or',
            msg.Node.NOT: 'not',
            msg.Node.COMP_EQ: '=',
            msg.Node.COMP_GE: '>=',
            msg.Node.COMP_GT: '>',
            msg.Node.COMP_LE: '<=',
            msg.Node.COMP_LT: '<',
            msg.Node.ARITH_MULT: '*',
            msg.Node.ARITH_DIV: '/',
            msg.Node.ARITH_ADD: '+',
            msg.Node.ARITH_SUB: '-',
        }

    # TODO: this function replaces the previous problem if present
    def addProblem(self, problem_str):
        # TODO: check if empty string
        try:
            self.problem = PDDLReader().parse_problem_string(self.domain_expert.getDomain(), problem_str)
            return True
        except:
            return False

    def _constructGoalExp(self, nodes, node_id):
        node = nodes[node_id]
        if node.node_type in [msg.Node.AND, msg.Node.OR, msg.Node.NOT]:
            sub_exp = ' '.join([self._constructGoalExp(nodes, child_id) for child_id in node.children])
            exp = f'({self.operators_map[node.node_type]} {sub_exp})'
        elif node.node_type == msg.Node.NUMBER:
            exp = f'{node.value}'
        elif node.node_type in [msg.Node.PREDICATE, msg.Node.FUNCTION]:
            param_exp = ' '.join([param.name for param in node.parameters])
            exp = f'({node.name} {param_exp})'
        elif node.node_type == msg.Node.EXPRESSION:
            sub_exp = ' '.join([self._constructGoalExp(nodes, child_id) for child_id in node.children])
            exp = f'({self.operators_map[node.expression_type]} {sub_exp})'
        else:
            raise Exception('node_type not valid')

        return exp
        
    # this function sobstitutes the goal if present
    def addProblemGoal(self, tree: msg.Tree):
        self.removeProblemGoal()

        try:
            types_map = {}
            for tt in self.problem.user_types_hierarchy.values():
                if len(tt) > 0:
                    for t in tt:
                        types_map[CaseInsensitiveToken(t.name)] = self.problem.user_type(t.name)

            nodes = dict([(node.node_id, node) for node in tree.nodes])
            goal_exp = self._constructGoalExp(nodes, 0)
            print(f'goal_exp::{goal_exp}')
            goal_res = nested_expr().setResultsName("goal").parse_string(goal_exp, parse_all=True)
            goalFNode = PDDLReader()._parse_exp(self.problem, None, types_map, {}, goal_res["goal"][0])
            print(f"goalFNode::{goalFNode}")
            self.problem.add_goal(
                goalFNode
            )

            return True
        except:
            return False

    # TODO: check if instance already exists 
    def addProblemInstance(self, instance: msg.Param):        
        try:
            type = self.problem.user_type(instance.type)
        except UPValueError:
            return False

        self.problem.add_object(up.model.Object(instance.name, type, self.problem.env))
        return True

    def addProblemPredicate(self, node: msg.Node):
        try:
            fluent = self.problem.fluent(node.name)
            objs = []
            for p in node.parameters:
                objs.append(self.problem.object(p.name))
        except UPValueError:
            return False
        
        self.problem.set_initial_value(fluent(*objs), True)
        return True

    def addProblemFunction(self, node):
        try:
            fluent = self.problem.fluent(node.name)
            objs = []
            for p in node.parameters:
                objs.append(self.problem.object(p.name))
        except UPValueError:
            return False
        
        self.problem.set_initial_value(fluent(*objs), self.problem.env.expression_manager.Real(Fraction(node.value)))
        return True

    def getProblemGoal(self):
        # TODO: consider timed goals
        tree = msg.Tree()
        tree.nodes = list()
        and_node = msg.Node()
        and_node.node_type = self.domain_expert.map_types[OperatorKind.AND]
        and_node.node_id = len(tree.nodes)
        and_node.children = list()
        tree.nodes.append(and_node)

        for goal in self.problem.goals:
            print(f"getProblemGoal::{goal}")
            node = self.domain_expert.constructTree(goal, tree.nodes)
            and_node.children.append(node.node_id)

        return tree

    def getProblemInstance(self, instance_name: str):
        for obj in self.problem.all_objects:
            if obj.name.lower() == instance_name.strip().lower():
                return self.domain_expert.constructParameters([obj])[0]
        return None

    def getProblemInstances(self):
        return self.domain_expert.constructParameters(self.problem.all_objects)

    def getProblemPredicate(self, predicate_exp: str):
        match = re.match(r"^\s*\(\s*([\w-]+)([\s\w-]*)\)\s*$", predicate_exp)
        if match is None:
            return None, None

        predicate_name = match.group(1)
        param_names = [arg.strip() for arg in match.group(2).split()]

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
                
                params_match = (len(predicate_msg.parameters) == len(param_names))
                if not params_match:
                    continue
                for j, param in enumerate(predicate_msg.parameters):
                    if param.name.lower() != param_names[j].lower():
                        params_match = False
                        break
                if params_match:
                    return fnode, predicate_msg

        return None, None

    def getProblemPredicates(self):
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
        return PDDLWriter(self.problem.clone()).get_problem() # TODO: clone should be avoided

    # TODO: remove useless and-node
    def removeProblemGoal(self):
        self.problem.clear_goals()
        return True

    # TODO: implement
    def clearProblemKnowledge(self):
        pass

    # TODO: type not checked
    def removeProblemInstance(self, instance: msg.Param):
        for obj in self.problem.all_objects:
            if obj.name.lower() == instance.name.lower():
                self.problem._objects.remove(obj)
                return True
        return False

    def removeProblemPredicate(self, node: msg.Node):
        params_str = ' '.join([p.name for p in node.parameters])
        predicate_exp = f"({node.name} {params_str})"
        predicate, predicate_msg = self.getProblemPredicate(predicate_exp)
        if predicate is None:
            return False
        else:
            del self.problem._initial_value[predicate]
            return True

    def removeProblemFunction(self, node: msg.Node):
        params_str = ' '.join([p.name for p in node.parameters])
        function_str = f"({node.name} {params_str})"
        function, function_msg = self.getProblemFunction(function_str)
        if function is None:
            return False
        else:
            del self.problem._initial_value[function]
            return True

    def existProblemPredicate(self, node: msg.Node):
        # TODO: params type not considered
        params_str = ' '.join([p.name for p in node.parameters])
        predicate_exp = f"({node.name} {params_str})"
        return self.getProblemPredicate(predicate_exp)[0] is not None

    def existProblemFunction(self, node: msg.Node):
        # TODO: params type not considered
        params_str = ' '.join([p.name for p in node.parameters])
        function_str = f"({node.name} {params_str})"
        return self.getProblemFunction(function_str)[0] is not None

    # TODO: test
    def updateProblemFunction(self, node):
        try:
            fluent = self.problem.fluent(node.name)
            objs = []
            for p in node.parameters:
                objs.append(self.problem.object(p.name))
        except UPValueError:
            return False
        
        self.problem.set_initial_value(fluent(*objs), self.problem.env.expression_manager.Real(Fraction(node.value)))
        return True

    # TODO: implement
    def isProblemGoalSatisfied(self):
        pass
