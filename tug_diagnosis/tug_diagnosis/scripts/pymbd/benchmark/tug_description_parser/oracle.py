from model import Model
from parser import SimpleParser
from pymbd.diagnosis.description import Description
from pymbd.util.two_way_dict import TwoWayDict
import time


class TUGDescriptionOracle(Description):
    
    def __init__(self, sisc_file, observations, **options):
        super(TUGDescriptionOracle, self).__init__([], **options)
        self.sisc_file = sisc_file
        self.comp_calls = 0
        self.check_calls = 0
        self.comp_time = 0
        self.check_time = 0
        self.scs = set()
        self.setup = False
        self.observations = observations
        self.net = None
        
    def setup_system_model(self):
        if not self.setup:
            self.setup = True
            self.net = Model(**self.options)
            # for c,i in enumerate(self.net.inputs):
            #     self.net.inputs[i] = self.inputs[c] == 1
            # for c,i in enumerate(self.net.outputs):
            #     self.net.outputs[i] = self.outputs[c] == 1
            self.nodes = TwoWayDict(dict(enumerate(["imu"])))
            
    def set_options(self, **options):
        super(TUGDescriptionOracle, self).set_options(**options)
        if self.net is not None:
            self.net.set_options(**options)
    
    def get_num_nodes(self):
        self.setup_system_model()
        return len(self.nodes)
        
    def get_conflict_set(self, h):
        self.setup_system_model()
        self.comp_calls += 1
        t0 = time.time()
        cs = self.net.calculate_conflicts(self.numbers_to_nodes(h))
        t1 = time.time()
        self.comp_time += t1-t0
        if cs:
            self.scs.add(frozenset(cs))
            return self.nodes_to_numbers(cs)
        else:
            return None
    
    def check_consistency(self, h):
        self.setup_system_model()
        self.check_calls += 1
        t0 = time.time()
        sat = self.net.check_consistency(self.numbers_to_nodes(h))
        t1 = time.time()
        self.check_time += t1-t0
        return sat

    def get_next_diagnosis(self, previous_diagnoses, max_card=None):
        self.setup_system_model()
        self.comp_calls += 1
        t0 = time.time()
#        print "get_next_diagnosis(%s,%d)"%(previous_diagnoses, max_card)
        diag = self.net.calculate_next_diagnosis(map(self.numbers_to_gates,previous_diagnoses), max_card=max_card)
#        print "solution:", diag
        t1 = time.time()
        self.comp_time += t1-t0
        if diag:
            return self.gates_to_numbers(diag)
        else:
            return None
        
    def get_all_diagnoses(self, previous_diagnoses, max_card=None, max_solutions=None):
        self.setup_system_model()
        self.comp_calls += 1
        t0 = time.time()
        diag = self.net.calculate_next_diagnosis(map(self.numbers_to_gates,previous_diagnoses), max_card=max_card, find_all_sols=True)
        t1 = time.time()
        self.comp_time += t1-t0
        if diag:
            return map(self.gates_to_numbers, diag)
        else:
            return None
        

    def finished(self):
        self.net.finished()
        
    # def gates_to_numbers(self, gates):
    #     return frozenset(map(lambda g: self.components.key(g), gates))
    #
    # def numbers_to_gates(self, numbers):
    #     return frozenset(map(lambda n: self.components[n], numbers))
    def nodes_to_numbers(self, nodes):
        return frozenset(map(lambda g: self.nodes.key(g), nodes))

    def numbers_to_nodes(self, numbers):
        return frozenset(map(lambda n: self.nodes[n], numbers))
