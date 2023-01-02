import unified_planning as up
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.model.action import InstantaneousAction
from unified_planning.model.operators import OperatorKind
from unified_planning.model import timing
# from plansys2_msgs import msg


reader = PDDLReader()
domain = reader.parse_problem(
    '/home/davide/plansys2_ws/src/plansys2_upf_problem_expert/tmp/domain2.pddl', None)

# print(domain.name)
# print(domain.user_types[0].name)
# print(domain._objects)
# print(list(domain.objects(domain.user_types[0])))
# print(domain.fluents)
# print(domain.fluent('test_predicate'))
# print(list(filter(lambda f: f.type.is_bool_type(), domain._fluents)))
# print(list(filter(lambda f: f.type.is_real_type(), domain._fluents)))
# print(type(list(domain.actions)[0]))
# print(list(domain.durative_actions))


# action = 'test_action1'
# instantaneous_actions = filter(lambda a: isinstance(a, InstantaneousAction), domain.actions)
# for a in instantaneous_actions:
#     if a.name == action:
#         print(a.effects)
#         for effect in a.effects:
#             print(effect)
#             print(effect.fluent)
#             print(effect.value)
#             print(effect.condition)
#             print(effect.kind)


# print(set(domain.actions))
# print(set(domain.durative_actions))
# print(set(domain.actions) - set(domain.durative_actions))
# for a in domain.durative_actions:
#     print(a.name)
#     print(a.parameters)
#     print(a.duration)
#     print(a.conditions)
#     for k,v in a.conditions.items():
#         print(type(k))
#         print(type(v[0]))
#         print(v[0].node_type)
#     print(a.effects)


# instantaneous_actions = list(filter(lambda a: isinstance(a, InstantaneousAction), domain.actions))
# action = instantaneous_actions[0]
# p = list(action.parameters)[0]
# # print(p.type)
# # print(p.type.father)
# for type in domain.user_types:
#     # print(type.father)
#     if type.father == p.type:
#         print(type)


# at_start = timing.TimeInterval(timing.TimepointKind.START, timing.TimepointKind.GLOBAL_END, is_right_open=True)
# print(at_start)
# print(list(domain.durative_actions)[0].effects)
# for time, effects in list(domain.durative_actions)[0].effects.items():
#     print(time == timing.StartTiming())
#     print(time)
#     print(effects)
#     print(effects[0].condition)


problem = PDDLReader().parse_problem("/home/davide/plansys2_ws/src/ros2_planning_system/plansys2_problem_expert/test/pddl/domain_simple.pddl", "/home/davide/plansys2_ws/src/ros2_planning_system/plansys2_problem_expert/test/pddl/problem_simple_1.pddl")
# print(problem.explicit_initial_values)
# for pred in problem.explicit_initial_values:
#     print(pred)
#     print(pred.args)
#     print([a.object().name for a in pred.args])
#     print(pred.fluent())
#     print(type(pred))
#     print(pred.is_fluent_exp())
#     print(pred.fluent().type.is_bool_type())



# import re
# predicate_str = "(robot_at )"
# match = re.match(r"^\s*\(\s*([\w-]+)([\s\w-]*)\)\s*$", predicate_str)
# print(match)
# print(match.group(0))
# print(match.group(1))
# params_name = [arg.strip() for arg in match.group(2).split()]
# print(params_name)

# print(problem.explicit_initial_values)
for fnode, value in problem.explicit_initial_values.items():
    # print(fnode)
    # print(fnode.is_fluent_exp())
    # print(fnode.fluent().type)
    # print(fnode, value)
    print(float(value.constant_value()))