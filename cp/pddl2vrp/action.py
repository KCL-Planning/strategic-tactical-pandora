#!/usr/bin/env python
# Four spaces as indentation [no tabs]

# Sept 19, 2017: Original source code from https://github.com/pucrs-automated-planning/pddl-parser
# Adapted by jcb@mie.utoronto.ca

class Action:

    def __init__(self, name, parameters, positive_preconditions, negative_preconditions, add_effects, del_effects, cost = 0):
        self.name = name
        self.parameters = parameters
        self.positive_preconditions = positive_preconditions
        self.negative_preconditions = negative_preconditions
        self.add_effects = add_effects
        self.del_effects = del_effects
        self.cost = cost

    def __str__(self):
        return 'action: ' + self.name + \
        '\n  parameters: ' + str(self.parameters) + \
        '\n  positive_preconditions: ' + str(self.positive_preconditions) + \
        '\n  negative_preconditions: ' + str(self.negative_preconditions) + \
        '\n  add_effects: ' + str(self.add_effects) + \
        '\n  del_effects: ' + str(self.del_effects) + \
        '\n  cost: ' + str(self.cost) + '\n'

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__
    
class Durative_Action(Action):
    
    def __init__(self, name, parameters, duration, at_start_pos_conditions, at_start_neg_conditions, at_end_pos_conditions, at_end_neg_conditions, over_all_pos_conditions, over_all_neg_conditions, add_at_start_effects, add_at_end_effects, del_at_start_effects, del_at_end_effects, cost = 0):
        super().__init__(name, parameters, [], [], [], [], cost)
        self.duration = duration
        self.at_start_pos_conditions = at_start_pos_conditions
        self.at_start_neg_conditions = at_start_neg_conditions
        self.at_end_pos_conditions = at_end_pos_conditions
        self.at_end_neg_conditions = at_end_neg_conditions
        self.over_all_pos_conditions = over_all_pos_conditions
        self.over_all_neg_conditions = over_all_neg_conditions
        self.add_at_start_effects = add_at_start_effects
        self.add_at_end_effects = add_at_end_effects
        self.del_at_start_effects = del_at_start_effects
        self.del_at_end_effects = del_at_end_effects

    def get_start_charge(self):
        charge_p = [int(p[2]) for p in self.at_start_pos_conditions if p[0] == ">=" and p[1][0] == "charge"]
        return charge_p[0]
    
    def get_end_charge(self):
        charge_p = [int(p[2]) for p in self.add_at_end_effects if p[0] == "assign" and p[1][0] == "charge"]
        return charge_p[0]

    def __str__(self):
        return 'action: ' + self.name + \
        '\n  parameters: ' + str(self.parameters) + \
        '\n  conditions: ' + \
        '\n  at_start:' + \
        '\n\t pos: ' + str(self.at_start_pos_conditions) + \
        '\n\t neg: ' + str(self.at_start_neg_conditions) + \
        '\n  at_end:' + \
        '\n\t pos: ' + str(self.at_end_pos_conditions) + \
        '\n\t neg: ' + str(self.at_end_neg_conditions) + \
        '\n  over_all:' + \
        '\n\t pos: ' + str(self.over_all_pos_conditions) + \
        '\n\t neg: ' + str(self.over_all_neg_conditions) + \
        '\n  add_effects: ' + str(self.add_effects) + \
        '\n  effects:' + \
        '\n  at_start:' + \
        '\n\t add: ' + str(self.add_at_start_effects) + \
        '\n\t del: ' + str(self.del_at_start_effects) + \
        '\n  at_end:' + \
        '\n\t add: ' + str(self.add_at_end_effects) + \
        '\n\t del: ' + str(self.del_at_end_effects) + \
        '\n  cost: ' + str(self.cost) + '\n'
