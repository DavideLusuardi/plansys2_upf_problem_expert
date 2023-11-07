from plansys2_msgs import msg
from plansys2_upf_problem_expert import ProblemExpert
from ament_index_python.packages import get_package_share_directory

import pytest


package_share_directory = get_package_share_directory('plansys2_upf_problem_expert')
domain_pddl_file = f'{package_share_directory}/test/pddl/domain1.pddl'
problem_pddl_file = f'{package_share_directory}/test/pddl/problem1.pddl'
domain2_pddl_file = f'{package_share_directory}/test/pddl/domain2.pddl'
problem2_pddl_file = f'{package_share_directory}/test/pddl/problem2.pddl'
with open(problem_pddl_file) as f:
    problem_str = f.read()
with open(problem2_pddl_file) as f:
    problem2_str = f.read()

@pytest.fixture
def problem() -> ProblemExpert:
    problem = ProblemExpert(domain_pddl_file)
    problem.addProblem(problem_str)
    return problem

@pytest.fixture
def problem2() -> ProblemExpert:
    problem = ProblemExpert(domain2_pddl_file)
    problem.addProblem(problem2_str)
    return problem


def test_addProblem():
    problem = ProblemExpert(domain_pddl_file)
    assert(problem.addProblem(problem_str))

def test_addProblemGoal(problem: ProblemExpert):
    node = msg.Node()
    node.node_type = msg.Node.PREDICATE
    node.name = "robot_at"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "leia"
    node.parameters[-1].type = "robot"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "kitchen"
    node.parameters[-1].type = "room"
    tree = msg.Tree()
    tree.nodes = [node]
    assert(problem.addProblemGoal(tree))

def test_addProblemInstance(problem: ProblemExpert):
    instance = msg.Param()
    instance.name = "my_object"
    instance.type = "robot"
    assert(problem.addProblemInstance(instance))

def test_addProblemPredicate(problem: ProblemExpert):
    node = msg.Node()
    node.node_type = msg.Node.PREDICATE
    node.name = "robot_at"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "leia"
    node.parameters[-1].type = "robot"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "chargingroom"
    node.parameters[-1].type = "room"
    assert(problem.addProblemPredicate(node))
    assert(problem.addProblemPredicate(node))

def test_addProblemFunction(problem: ProblemExpert):
    node = msg.Node()
    node.node_type = msg.Node.FUNCTION
    node.name = "distance_travelled"
    node.value = float(1)
    assert(problem.addProblemFunction(node))
    assert(problem.addProblemFunction(node))

def test_getProblemGoal(problem: ProblemExpert):
    goal = problem.getProblemGoal()
    assert(len(goal.nodes)==2)

def test_getProblemInstance(problem: ProblemExpert):
    instance = problem.getProblemInstance('leia')
    assert(instance is not None)

def test_getProblemInstances(problem: ProblemExpert):
    instances = problem.getProblemInstances()
    assert(len(instances)==7)

def test_getProblemPredicate(problem: ProblemExpert):
    predicate, predicate_msg = problem.getProblemPredicate('(connected entrance dinning)')
    assert(predicate is not None and predicate_msg is not None)

def test_getProblemPredicates(problem: ProblemExpert):
    predicates = problem.getProblemPredicates()
    assert(len(predicates)==13)

def test_getProblemFunction(problem: ProblemExpert):
    function, function_msg = problem.getProblemFunction('(distance_travelled)')
    assert(function is not None and function_msg is not None)

def test_getProblemFunctions(problem: ProblemExpert):
    functions = problem.getProblemFunctions()
    assert(len(functions)==1)

def test_getProblem(problem: ProblemExpert):
    assert(problem.getProblem() is not None)

def test_removeProblemGoal(problem: ProblemExpert):
    assert(problem.removeProblemGoal())

def test_clearProblemKnowledge(problem: ProblemExpert):
    assert(problem.clearProblemKnowledge())

def test_removeProblemInstance(problem: ProblemExpert):
    instance = msg.Param()
    instance.name = 'leia'
    instance.type = 'robot'
    assert(problem.removeProblemInstance(instance))
    assert(not problem.removeProblemInstance(instance))

def test_removeProblemPredicate(problem: ProblemExpert):
    node = msg.Node()
    node.node_type = msg.Node.PREDICATE
    node.name = "robot_at"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "leia"
    node.parameters[-1].type = "robot"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "entrance"
    node.parameters[-1].type = "room"
    assert(problem.removeProblemPredicate(node))
    assert(not problem.removeProblemPredicate(node))

    pp = problem.getProblemPredicates()
    for p in pp:
        assert(problem.removeProblemPredicate(p))

def test_removeProblemFunction(problem: ProblemExpert):
    node = msg.Node()
    node.node_type = msg.Node.FUNCTION
    node.name = "distance_travelled"
    assert(problem.removeProblemFunction(node))
    assert(not problem.removeProblemFunction(node))

    ff = problem.getProblemFunctions()
    for f in ff:
        assert(problem.removeProblemFunction(f))

def test_existProblemPredicate(problem: ProblemExpert):
    node = msg.Node()
    node.node_type = msg.Node.PREDICATE
    node.name = "robot_at"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "leia"
    node.parameters[-1].type = "robot"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "entrance"
    node.parameters[-1].type = "room"
    assert(problem.existProblemPredicate(node))

def test_existProblemFunction(problem: ProblemExpert):
    node = msg.Node()
    node.node_type = msg.Node.FUNCTION
    node.name = "distance_travelled"
    assert(problem.existProblemFunction(node))

def test_updateProblemFunction(problem: ProblemExpert):
    node = msg.Node()
    node.node_type = msg.Node.FUNCTION
    node.name = "distance_travelled"
    node.value = float(10)
    assert(problem.updateProblemFunction(node))

def test_isProblemGoalSatisfied(problem2: ProblemExpert):
    assert(not problem2.isProblemGoalSatisfied())
    
    node = msg.Node()
    node.node_type = msg.Node.PREDICATE
    node.name = "robot_at"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "leia"
    node.parameters[-1].type = "robot"
    node.parameters.append(msg.Param())
    node.parameters[-1].name = "entrance"
    node.parameters[-1].type = "room"
    tree = msg.Tree()
    tree.nodes = [node]
    problem2.addProblemGoal(tree)
    assert(problem2.isProblemGoalSatisfied())

