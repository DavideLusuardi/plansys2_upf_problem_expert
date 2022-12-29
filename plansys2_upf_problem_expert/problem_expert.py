from plansys2_msgs import msg

import unified_planning as up
from unified_planning.io import PDDLReader
from unified_planning.model.action import InstantaneousAction
from unified_planning.model.operators import OperatorKind
from unified_planning.model.effect import EffectKind
from unified_planning.model import timing

from plansys2_upf_domain_expert import DomainExpert

from typing import List, Dict
import tempfile
import os
import re


class ProblemExpert():

    def __init__(self, domain_filename: str):
        # self.domain = PDDLReader().parse_problem(domain_filename, None)
        self.problem = None
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


    def addProblem(self, problem_str):
        # TODO: check if empty string
        problem = self.parseProblem(problem_str)
        return problem is not None

    def addProblemGoal(self):
        pass

    def addProblemInstance(self):
        pass

    def addProblemPredicate(self):
        pass

    def addProblemFunction(self):
        pass

    def getProblemGoal(self):
        if self.problem is None:
            return []
        
        # TODO: to consider timed goals
        # TODO: add and-node as root
        tree = msg.Tree()
        tree.nodes = list()
        for goal in self.problem.goals:
            self.domain_expert.constructTree(goal, tree.nodes)

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

        for i, predicate_fnode in enumerate(self.problem.explicit_initial_values):
            assert(predicate_fnode.is_fluent_exp())
            predicate = predicate_fnode.fluent()
            assert(predicate.type.is_bool_type())

            if predicate.name == predicate_name:
                predicate_msg = msg.Node()
                predicate_msg.node_type = msg.Node.PREDICATE
                predicate_msg.node_id = i
                predicate_msg.name = predicate.name
                params_map = dict([(p.name, predicate_fnode.args[j].object().name) for j,p in enumerate(predicate.signature)])
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
        for i, predicate_fnode in enumerate(self.problem.explicit_initial_values):
            assert(predicate_fnode.is_fluent_exp())
            predicate = predicate_fnode.fluent()
            assert(predicate.type.is_bool_type())
            
            predicate_msg = msg.Node()
            predicate_msg.node_type = msg.Node.PREDICATE
            predicate_msg.node_id = i
            predicate_msg.name = predicate.name
            params_map = dict([(p.name, predicate_fnode.args[j].object().name) for j,p in enumerate(predicate.signature)])
            predicate_msg.parameters = self.domain_expert.constructParameters(predicate.signature, params_map)
            predicates.append(predicate_msg)
        return predicates

    def getProblemFunction(self):
        pass

    def getProblemFunctions(self):
        pass

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

    def existProblemPredicate(self):
        pass

    def existProblemFunction(self):
        pass

    def updateProblemFunction(self):
        pass

    def isProblemGoalSatisfied(self):
        pass

