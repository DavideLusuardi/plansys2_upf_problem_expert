# UPF Problem Expert

The UPF Problem Expert module is responsible for maintaining the instances, predicates and goals of the PDDL problem.

The main class is [`ProblemExpertNode`](plansys2_upf_problem_expert/problem_expert_node.py). `ProblemExpertNode` simulates a `rclcpp_lifecycle::LifecycleNode` and in its configuration phase reads the `model_file` parameter, which contains the .pddl file from which to read the model.

The class responsible for maintaining this problem instance is [`ProblemExpert`](plansys2_upf_problem_expert/problem_expert.py), which is independent of ROS2.

The UPF Problem Expert is dynamic and volatile, accessing its functionality through ROS2 services. 

Every update in the Problem, is notified publishing a `std_msgs::msg::Empty` in `/problem_expert/update_notify`. It helps other modules and applications to be aware of updates, being not necessary to do polling to check it. For every update, a `plansys2_msgs::msg::Knowledge` is published in `/problem_expert/knowledge`.

## Services

- `/problem_expert/add_problem` [`plansys2_msgs::srv::AddProblem`]
- `/problem_expert/add_problem_goal` [`plansys2_msgs::srv::AddProblemGoal`]
- `/problem_expert/add_problem_instance` [`plansys2_msgs::srv::AffectParam`]
- `/problem_expert/add_problem_predicate` [`plansys2_msgs::srv::AffectNode`]
- `/problem_expert/add_problem_function` [`plansys2_msgs::srv::AffectNode`]
- `/problem_expert/get_problem_goal` [`plansys2_msgs::srv::GetProblemGoal`]
- `/problem_expert/get_problem_instance` [`plansys2_msgs::srv::GetProblemInstanceDetails`]
- `/problem_expert/get_problem_instances` [`plansys2_msgs::srv::GetProblemInstances`]
- `/problem_expert/get_problem_predicate` [`plansys2_msgs::srv::GetNodeDetails`]
- `/problem_expert/get_problem_predicates` [`plansys2_msgs::srv::GetStates`]
- `/problem_expert/get_problem_function` [`plansys2_msgs::srv::GetNodeDetails`]
- `/problem_expert/get_problem_functions` [`plansys2_msgs::srv::GetStates`]
- `/problem_expert/get_problem` [`plansys2_msgs::srv::GetProblem`]
- `/problem_expert/remove_problem_goal` [`plansys2_msgs::srv::RemoveProblemGoal`]
- `/problem_expert/clear_problem_knowledge` [`plansys2_msgs::srv::ClearProblemKnowledge`]
- `/problem_expert/remove_problem_instance` [`plansys2_msgs::srv::AffectParam`]
- `/problem_expert/remove_problem_predicate` [`plansys2_msgs::srv::AffectNode`]
- `/problem_expert/remove_problem_function` [`plansys2_msgs::srv::AffectNode`]
- `/problem_expert/exist_problem_predicate` [`plansys2_msgs::srv::ExistNode`]
- `/problem_expert/exist_problem_function` [`plansys2_msgs::srv::ExistNode`]
- `/problem_expert/update_problem_function` [`plansys2_msgs::srv::AffectNode`]
- `/problem_expert/is_problem_goal_satisfied` [`plansys2_msgs::srv::IsProblemGoalSatisfied`]

## Published topics

- `/problem_expert/update_notify` [`std_msgs::msg::Empty`]
- `/problem_expert/knowledge` [`plansys2_msgs::msg::Knowledge`]