from model import Model
from pymbd.diagnosis.description import Description
from pymbd.util.two_way_dict import TwoWayDict
import time


class TUGDescriptionOracle(Description):
    
    def __init__(self, configs, observations, **options):
        super(TUGDescriptionOracle, self).__init__([], **options)
        self.configs = configs
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
            self.net = Model(self.configs, **self.options)
            self.net.set_observations(self.observations)
            self.nodes = TwoWayDict(dict(enumerate(self.net.temp_nodes)))
            
    def set_options(self, **options):
        super(TUGDescriptionOracle, self).set_options(**options)
        if self.net is not None:
            self.net.set_options(**options)

    def is_real_node(self, node_name):
        return self.net.is_real_node(node_name)
    
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


    def finished(self):
        self.net.finished()

    def nodes_to_numbers(self, nodes):
        return frozenset(map(lambda g: self.nodes.key(g), nodes))

    def numbers_to_nodes(self, numbers):
        return frozenset(map(lambda n: self.nodes[n], numbers))
