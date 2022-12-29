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
            self.domain_expert.constructTree(goal, tree.nodes, None)

        return tree

    def getProblemInstance(self):
        pass

    def getProblemInstances(self):
        pass

    def getProblemPredicate(self):
        pass

    def getProblemPredicates(self):
        pass

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

