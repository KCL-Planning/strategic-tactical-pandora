#!/usr/bin/env python
# Four spaces as indentation [no tabs]

# Sept 19, 2017: Original source code from https://github.com/pucrs-automated-planning/pddl-parser
# Adapted by jcb@mie.utoronto.ca

import re
from action import Action
from action import Durative_Action

class PDDL_Parser:
        
    # ------------------------------------------
    # Tokens
    # ------------------------------------------

    def scan_tokens(self, filename):
        with open(filename,'r') as f:
            # Remove single line comments
            str = re.sub(r';.*$', '', f.read(), flags=re.MULTILINE).lower()
        # Tokenize
        stack = []
        list = []
        for t in re.findall(r'[()]|[^\s()]+', str):
            if t == '(':
                stack.append(list)
                list = []
            elif t == ')':
                if stack:
                    l = list
                    list = stack.pop()
                    list.append(l)
                else:
                    raise Exception('Missing open parentheses')
            else:
                list.append(t)
        if stack:
            raise Exception('Missing close parentheses')
        if len(list) != 1:
            raise Exception('Malformed expression')
        return list[0]

    #-----------------------------------------------
    # Parse domain
    #-----------------------------------------------

    def parse_domain(self, domain_filename):
        tokens = self.scan_tokens(domain_filename)
        if type(tokens) is list and tokens.pop(0) == 'define':
            self.domain_name = 'unknown'
            self.actions = []
            self.dur_actions = []
            self.types = []
            while tokens:
                group = tokens.pop(0)
                t = group.pop(0)
                if   t == 'domain':
                    self.domain_name = group[0]
                elif t == ':requirements':
                    pass # TODO
                elif t == ':predicates':
                    pass # TODO
                elif t == ':types':
                    self.types = group
                    #print(self.types)
                elif t == ':action':
                    self.parse_action(group)
                elif t == ':durative-action':
                    self.parse_durative_action(group)
                else: print("# " + str(t) + ' is not recognized in domain')
        else:
            raise 'File ' + domain_filename + ' does not match domain pattern'

    #-----------------------------------------------
    # Parse action
    #-----------------------------------------------

    def parse_action(self, group):
        name = group.pop(0)
        if not type(name) is str:
            raise Exception('Action without name definition')
        for act in self.actions:
            if act.name == name:
                raise Exception('Action ' + name + 'redefined')
        parameters = []
        positive_preconditions = []
        negative_preconditions = []
        add_effects = []
        del_effects = []
        while group:
            t = group.pop(0)
            if t == ':parameters':
                if not type(group) is list:
                    raise Exception('Error with '+ name + ' parameters')
                parameters = group.pop(0)
            elif t == ':precondition':
                self.split_propositions(group.pop(0), positive_preconditions, negative_preconditions, name, ' preconditions')
            elif t == ':effect':
                self.split_propositions(group.pop(0), add_effects, del_effects, name, ' effects')
            else: print(str(t) + ' is not recognized in action')
        self.actions.append(Action(name, parameters, positive_preconditions, negative_preconditions, add_effects, del_effects))

    #-----------------------------------------------
    # Parse durative-action
    #-----------------------------------------------

    def parse_durative_action(self, group):
        name = group.pop(0)
        if not type(name) is str:
            raise Exception('Action without name definition')
        for act in self.dur_actions:
            if act.name == name:
                raise Exception('Action ' + name + 'redefined')
        parameters = []
        duration = []
        #positive_preconditions = []
        #negative_preconditions = []
        
        at_start_neg_conditions = []
        at_start_pos_conditions = []
        at_end_neg_conditions = []
        at_end_pos_conditions = []
        over_all_neg_conditions = []
        over_all_pos_conditions = []
        
        add_at_start_effects = []
        add_at_end_effects = []
        del_at_start_effects = []
        del_at_end_effects = []
        
        while group:
            t = group.pop(0)
            if t == ':parameters':
                if not type(group) is list:
                    raise Exception('Error with '+ name + ' parameters')
                parameters = group.pop(0)
            elif t == ':duration':
                if not type(group) is list:
                    raise Exception('Error with '+ name + ' duration')
                duration = group.pop(0)  
                #print("dur:", duration)
            elif t == ':condition':
                self.split_durative_propositions(group.pop(0), at_start_pos_conditions, at_start_neg_conditions, at_end_pos_conditions, at_end_neg_conditions, over_all_pos_conditions, over_all_neg_conditions, name, ' conditions')
            elif t == ':effect':
                self.split_durative_propositions(group.pop(0), add_at_start_effects, del_at_start_effects, add_at_end_effects, del_at_end_effects, [], [], name, ' effects')
            else: print(str(t) + ' is not recognized in durative-action')
            
        self.dur_actions.append(Durative_Action(name, parameters, duration, at_start_pos_conditions, at_start_neg_conditions, at_end_pos_conditions, at_end_neg_conditions, over_all_pos_conditions, over_all_neg_conditions, add_at_start_effects, add_at_end_effects, del_at_start_effects, del_at_end_effects))

    #-----------------------------------------------
    # Parse problem
    #-----------------------------------------------

    def parse_problem(self, problem_filename):
        tokens = self.scan_tokens(problem_filename)
        if type(tokens) is list and tokens.pop(0) == 'define':
            self.problem_name = 'unknown'
            self.objects = []
            self.state = []
            self.positive_goals = []
            self.negative_goals = []
            while tokens:
                group = tokens.pop(0)
                t = group[0]
                if   t == 'problem':
                    self.problem_name = group[-1]
                elif t == ':domain':
                    if self.domain_name != group[-1]:
                        raise Exception('Different domain specified in problem file')
                elif t == ':requirements':
                    pass # TODO
                elif t == ':objects':
                    group.pop(0)
                    self.objects = group
                elif t == ':init':
                    group.pop(0)
                    self.state = group
                elif t == ':goal':
                    self.split_propositions(group[1], self.positive_goals, self.negative_goals, '', 'goals')
                else: print("# " + str(t) + ' is not recognized in problem')

    #-----------------------------------------------
    # Split propositions
    #-----------------------------------------------

    def split_propositions(self, group, pos, neg, name, part):
        if not type(group) is list:
            raise Exception('Error with '+ name + part)
        if group[0] == 'and':
            group.pop(0)
        else:
            group = [group]
        for proposition in group:
            if proposition[0] == 'not':
                if len(proposition) != 2:
                    raise Exception('Error with ' + name + ' negative' + part)
                neg.append(proposition[-1])
            else:
                pos.append(proposition)
                
    #-----------------------------------------------
    # Split durative-propositions
    #-----------------------------------------------

    def split_durative_propositions(self, group, at_start_pos, at_start_neg, at_end_pos, at_end_neg, over_all_pos, over_all_neg, name, part):
        if not type(group) is list:
            raise Exception('Error with '+ name + part)
        if group[0] == 'and':
            group.pop(0)
        else:
            group = [group]
            
        for proposition in group:
            #print("Prop:",proposition)
            
            if proposition[0] == 'at':
                if proposition[1] == "start":
                    #print("Callng split_propositions", proposition[2])
                    self.split_propositions(proposition[2], at_start_pos, at_start_neg, name, ' at start conditions')
                elif proposition[1] == "end":
                    self.split_propositions(proposition[2], at_end_pos, at_end_neg, name, ' at end conditions')
            elif proposition[0] == "over" and proposition[1] == "all":
                self.split_propositions(proposition[2], over_all_pos, over_all_neg, name, ' over all conditions')
            else:
                print("# " + str(proposition) + ' is not recognized in durative-action')
              
        #print("At start pos:", at_start_pos)
        #print("At start neg:", at_start_neg)
        #print("At end pos:", at_end_pos)
        #print("At end neg:", at_end_neg)
        #print("Over all pos:", over_all_pos)
        #print("Over all neg:", over_all_neg)
        
# ==========================================
# Main
# ==========================================
if __name__ == '__main__':
    import sys
    import pprint
    domain = sys.argv[1]
    problem = sys.argv[2]
    parser = PDDL_Parser()
    print('----------------------------')
    pprint.pprint(parser.scan_tokens(domain))
    print('----------------------------')
    pprint.pprint(parser.scan_tokens(problem))
    print('----------------------------')
    parser.parse_domain(domain)
    parser.parse_problem(problem)
    print('Domain name:' + parser.domain_name)
    for act in parser.actions:
        print(act)
    print('----------------------------')
    print('Problem name: ' + parser.problem_name)
    print('Objects: ' + str(parser.objects))
    print('State: ' + str(parser.state))
    print('Positive goals: ' + str(parser.positive_goals))
    print('Negative goals: ' + str(parser.negative_goals))