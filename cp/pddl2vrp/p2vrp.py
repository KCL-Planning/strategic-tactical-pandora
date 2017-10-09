#!/Users/jcb/anaconda/bin/python3

import sys
import pprint

from pddl import PDDL_Parser
from action import Durative_Action

class VRP:
    '''Represent a parsed Pandora PDDL file and output a VRP formatted 
       for input into the CP solver.
       This is a domain specific implementation, assuming that we are 
       solving a Pandora domain and making use of string matching on 
       specific names.
    '''
    
    def __init__(self, name):
        self.name = name
        self.vehicles = []
        self.waypoints = []
        self.missions = []
        self.mission_delta_charge = ""
        self.recharges = []
        self.delta_charge_coeff = 0
        self.travel_duration_coeff = 0

    def set_objects(self, objects):
        '''(self, list) -> None
        Parse the list of objects coming from PDDL into vehicles, waypoints,
        and missions.
        '''
        # objects is a flat list with the following form:
        # ["obj1_name", "obj2_name", "-", "type1", "obj3_name", "-", "type2", ...]
        # the '-' character is used to separate the objects from their types
        
        # copy list so as to not destroy parser content
        my_objs = objects[:]
        while(my_objs):
            i = my_objs.index('-')
            obj_of_type = my_objs[:i+2]
            my_objs = my_objs[i+2:]
            self.process_type(obj_of_type)
            #print(obj_of_type)
            #print(my_objs)

    def set_recharges(self, l):
        self.recharges = [Recharge(p[1]) for p in l]
            
    def process_type(self, obj_of_type):
        ''' (self, list) -> None
        Process a list of objects of a given type and store list of objects
        as appropriate
        '''
        type = obj_of_type[-1]
        objs = obj_of_type[:-2]
        if type == "vehicle":
            self.vehicles = [Vehicle(v) for v in objs]
        elif type == "waypoint":
            self.waypoints = [Waypoint(w) for w in objs]
        elif type == "mission":
            self.missions = [Mission(m) for m in objs]
        else:
            print("# Error - unrecognized type", type, "- ignored")

    def parse_mission_delta_charge(self, dur_actions):
        ''' (list) -> float
        If mission_delta_charge has already been changed from its default, then
        that value is returned. Otherwise, set the mission_delta_charge based
        on the complete_mission durative actio and return the value
        '''
        if self.mission_delta_charge == "":
            # not yet parsed
            # get first action with name "complete_mission"
            complete_mission = next(act for i,act in enumerate(dur_actions) if act.name == "complete_mission")
            decrease_iter = (p for i,p in enumerate(complete_mission.add_at_end_effects) if p[0] == "decrease")
            for p in decrease_iter:
                assert isinstance(p[1],list)
                if p[1][0] == "charge":
                    self.mission_delta_charge = p[2]
                    return self.mission_delta_charge
            
        return self.mission_delta_charge

    def parse_recharges(self, dur_actions):
        ''' (list) -> None
        Parse the list of recharging actions, pulling out the relevant info
        '''
        for a in dur_actions:
            if a.name == "dock_auv":
                dock_duration = int(a.duration[2])
                min_start_charge = a.get_start_charge()
            elif a.name == "recharge":
                duration = int(a.duration[2])
                end_charge = a.get_end_charge()
            elif a.name == "undock_auv":
                undock_duration = int(a.duration[2])
                
        for r in self.recharges:
            r.dock_duration = dock_duration
            r.duration = duration
            r.undock_duration = undock_duration
            r.min_start_charge = min_start_charge
            r.end_charge = end_charge
            
    def set_travel_delta_charge(self, dur_actions):
        travel = next(act for i,act in enumerate(dur_actions) if act.name == "do_hover")
        decrease_iter = (p for i,p in enumerate(travel.add_at_end_effects) if p[0] == "decrease")
        for p in decrease_iter:
            assert isinstance(p[1],list)
            if p[1][0] == "charge":
                delta_formula = p[2]
                assert delta_formula[0] == "*"
                assert delta_formula[1][0] == "distance"
                self.delta_charge_coeff = float(delta_formula[2])
                #return self.mission_delta_charge

    def set_travel_duration_coeff(self, dur_actions):
        travel = next(act for i,act in enumerate(dur_actions) if act.name == "do_hover")
        d = travel.duration

        assert isinstance(d[2], list)
        assert isinstance(d[2][1], list)
        if d[2][1][0] == "distance":
            delta_formula = d[2]
            assert delta_formula[0] == "*"
            self.travel_duration_coeff = float(delta_formula[2])

        #print(str(self.travel_coeff))

        
    def print_VRP_format(self):
        '''(None) -> None
        Outputs the object in VRP format.
        '''
        self.print_VRP_vehicles()
        self.print_VRP_missions()
        self.print_VRP_recharges()
        self.print_VRP_distances()
        self.print_VRP_travel()


    def print_VRP_vehicles(self):
        
        print("# Vehicles")
        print("nb_vehicles " + str(len(self.vehicles)))
        for v in self.vehicles:
            print(v)
              

    def print_VRP_missions(self):
            
        print("# Missions")
        print("nb_missions", len(self.missions))
        for m in self.missions:
            print(m)

    def print_VRP_recharges(self):
            
        print("# Recharging")
        print("nb_recharges", len(self.recharges))
        for r in self.recharges:
            print(r)
              

    def print_VRP_distances(self):
            
        print("# Distances")
        nb_pairs = 0
        for w in self.waypoints:
            nb_pairs += len(w.distances)
        print("nb_distances " + str(nb_pairs))
        
        for w in self.waypoints:
            w.print_VRP_format()
            
        
    def print_VRP_travel(self):
            
        print("# Travel")
        print("delta_charge_coeff", self.delta_charge_coeff)
        print("travel_duration_coeff", self.travel_duration_coeff)
    
         
class Vehicle:
    
    def __init__(self, name):
        self.name = name
        self.start_charge = 0
        self.start_waypoint = ""
        
    def read_state(self, state):
        ''' (list) -> None
        State is a list of proposition about the state that are relevant to 
        the current object (i.e., contain the name of the object at some
        level of nesting). The function populates the object with the information
        in the state
        '''
        for p in state:
            if p[0] == "at":
                self.start_waypoint = p[-1]
            elif p[0] == "=":
                assert p[1][0] == "charge"
                self.start_charge = float(p[-1])
                
    def __str__(self):
        return self.name + " " + str(self.start_charge) + " " + self.start_waypoint
          
class Waypoint:
    def __init__(self, name):
        self.name = name
        # distances from self to each of the other waypoints it is connected to
        # distances are not necessarily symmetric. 
        self.distances = []

    def read_state(self, state):
        ''' (list) -> None
        State is a list of proposition about the state that are relevant to 
        the current object (i.e., contain the name of the object at some
        level of nesting). The function populates the object with the information
        in the state
        '''
        for p in state:
            #print(p)
            # ignoring everything but the distance from current object to other
            # waypoint
            if p[0] == "=":
                distance_p = p[1]
                assert distance_p[0] == "distance"
                if distance_p[1] == self.name:
                    self.distances.append([distance_p[2], float(p[-1])])

    def print_VRP_format(self):
        for d in self.distances:
            print(self.name, d[0], d[1])
            
    def __str__(self):
        return self.name + " " + str(self.distances)

class Mission:
    def __init__(self, name):
        self.name = name
        self.duration = 0
        self.delta_charge = 0
        self.start_waypoint = ""
        self.deadline = -1
        
    def read_state(self, state):
        ''' (list) -> None
        State is a list of proposition about the state that are relevant to
        the current object (i.e., contain the name of the object at some
        level of nesting). The function populates the object with the information
        in the state
        '''
        for p in state:
            #print(p)
            if p[0] == "in":
                # specifies starting waypoint for mission
                assert p[1] == self.name
                self.start_waypoint = p[-1]
            elif p[0] == "at":
                # specifies deadline
                assert isinstance(p[2], list)
                assert p[2][0] == "not"
                assert isinstance(p[2][1], list)
                assert p[2][1][0] == "active"
                if p[1] != "99999":
                    self.deadline = float(p[1])
            elif p[0] == "=":
                # specifies duration
                assert isinstance(p[1], list)
                assert p[1][0] == "mission_duration"
                self.duration = float(p[-1])
                    
                #distance_p = p[1]
                #
                #if distance_p[1] == self.name:
                    #self.distances.append([distance_p[2], float(p[-1])])

    def set_delta_charge(self, p):
        ''' (list) -> None
        Takes the input from the domain file defining the operation associated
        with the decrease of the charge when a mission is completed
        '''
        if p[0] == "mission_duration":
            assert self.duration != 0
            self.delta_charge = self.duration
        else:
            print("Complicated delta_charge. TO DO:", p)
            
    def __str__(self):
        return self.name + " " + str(self.duration) + " " + str(self.delta_charge) + " " + self.start_waypoint + " " + str(self.deadline)
        
class Recharge:
    def __init__(self, location):
        self.waypoint = location
        self.dock_duration = 0
        self.recharge_duration = 0
        self.undock_duration = 0
        self.min_start_charge = 0
        self.end_charge = -1
        
    def __str__(self):
        dock_str = "dock " + str(self.dock_duration) + " " + str(self.min_start_charge)
        recharge_str = "recharge " + self.waypoint + " " + str(self.duration) + " " + str(self.end_charge)
        undock_str = "undock " + str(self.undock_duration)
        return dock_str + "\n" + recharge_str + "\n" + undock_str

    
        
def deep_find(l, s):
    '''(list, string) -> list
    Searches through an arbitrarily nested list for v. Returns true if 
    l contains s anywhere  
    '''
    for element in l:
        if isinstance(element, list):
            if deep_find(element, s):
                return True
        elif element == s:
            return True
    return False    

# ==========================================
# Main
# ==========================================
if __name__ == '__main__':
    domain = sys.argv[1]
    problem = sys.argv[2]
    parser = PDDL_Parser()
    parser.parse_domain(domain)
    
    #for act in parser.dur_actions:
    #    print(act)

    parser.parse_problem(problem)
    print('# Domain name:' + parser.domain_name)
    print('# Problem name: ' + parser.problem_name)
    print('# ----------------------------')

    vrp = VRP(parser.problem_name)
    vrp.set_objects(parser.objects)

    # get recharging locations
    vrp.set_recharges([entry for entry in parser.state if entry[0] == "recharge_at"])
    vrp.parse_recharges([a for a in parser.dur_actions if a.name == "dock_auv" or a.name == "recharge" or a.name == "undock_auv"])
    
    #print('Objects: ' + str(parser.objects))
    #print('State: ' + str(parser.state))
    
    for v in vrp.vehicles:
        v.read_state([entry for entry in parser.state if deep_find(entry,v.name)])

    for m in vrp.missions:
        m.read_state([entry for entry in parser.state if deep_find(entry,m.name)])
        m.set_delta_charge(vrp.parse_mission_delta_charge(parser.dur_actions))

    for w in vrp.waypoints:
        w.read_state([entry for entry in parser.state if deep_find(entry,w.name)])


    vrp.set_travel_delta_charge(parser.dur_actions)
    vrp.set_travel_duration_coeff(parser.dur_actions)

    vrp.print_VRP_format()
    
    #print('----------------------------')
    #pprint.pprint(parser.scan_tokens(domain))
    #print('----------------------------')
    #pprint.pprint(parser.scan_tokens(problem))
    #print('----------------------------')
    #parser.parse_domain(domain)
    #parser.parse_problem(problem)
    #print('Domain name:' + parser.domain_name)
    #for act in parser.actions:
        #print(act)
    #print('----------------------------')
    #print('Problem name: ' + parser.problem_name)
    #print('Objects: ' + str(parser.objects))
    #print('State: ' + str(parser.state))
    #print('Positive goals: ' + str(parser.positive_goals))
    #print('Negative goals: ' + str(parser.negative_goals))
