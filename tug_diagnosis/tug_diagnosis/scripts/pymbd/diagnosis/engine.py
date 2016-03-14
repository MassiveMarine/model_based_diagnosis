import sys
import time

from ..algorithm.hsdag import Greiner, GreinerO, GreinerN
from ..algorithm.hst import HST, HSTNode, HSTO
from ..algorithm.node import Node
from ..diagnosis.result import Result


def merge(opts1, opts2):
    return dict(opts1.items() + opts2.items())


ENGINES = {
    'hst-picosat': lambda d, o: PyHSTEngine(d, merge({'sat_solver': 'picosat', 'cache': False}, o)),
    'hst-cache-picosat': lambda d, o: PyHSTEngine(d, merge({'sat_solver': 'picosat', 'cache': True}, o)),
    'hst-ci-picosat': lambda d, o: PyHSTOEngine(d, merge({'sat_solver': 'picosat', 'cache': False}, o)),
    'hst-ci-cache-picosat': lambda d, o: PyHSTOEngine(d, merge({'sat_solver': 'picosat', 'cache': True}, o)),
    'hsdag-picosat': lambda d, o: PyHsDagEngine(d, merge({'sat_solver': 'picosat', 'cache': False}, o)),
    'hsdag-cache-picosat': lambda d, o: PyHsDagEngine(d, merge({'sat_solver': 'picosat', 'cache': True}, o)),
    'hsdag-ci-picosat': lambda d, o: PyHsDagOEngine(d, merge({'sat_solver': 'picosat', 'cache': False}, o)),
    'hsdag-ci-cache-picosat': lambda d, o: PyHsDagOEngine(d, merge({'sat_solver': 'picosat', 'cache': True}, o)),
    'hsdag-sicf-picosat': lambda d, o: PyHsDagNEngine(d, merge({'sat_solver': 'picosat', 'cache': False}, o)),
    'hsdag-sicf-cache-picosat': lambda d, o: PyHsDagNEngine(d, merge({'sat_solver': 'picosat', 'cache': True}, o)),
}


def hittingsets_from_tree(hstree, card=(1, sys.maxint)):
    hs = set()
    min_card = card[0]
    max_card = card[1]
    if not min_card:
        min_card = 1
    if not max_card:
        max_card = sys.maxint
    if len(hstree.checked_nodes) > 0:
        cardinalities = hstree.checked_nodes.keys()
        for cardinality in xrange(min_card, min(max(cardinalities), max_card) + 1):
            if cardinality in cardinalities:
                for node in hstree.checked_nodes[cardinality]:
                    hs.add(node.h)
    return hs


class Engine(object):
    '''
    Encapsules an algorithm that does the actual hitting set computation.
    This implementation does nothing, derived classes must implement at least
    the start() method and place a Result object in self.result at the end
    '''

    def __init__(self, description, options):
        '''
        Construct a hitting set computation engine from the problem description.
        An options hash specific to the algorithm  may be given as second parameter.
        '''
        self.description = description
        self.options = options
        self.result = None
        self.handle = None

    def start(self):
        pass

    def get_result(self):
        return self.result

    def set_options(self, options):
        self.options.update(options)


class PyHsDagEngine(Engine):
    def __init__(self, description, options):
        super(PyHsDagEngine, self).__init__(description, options)
        o = self.options
        o['min_card'] = o.get('min_card', 1)
        o['max_card'] = o.get('max_card', sys.maxint)
        o['max_time'] = o.get('max_time', None)
        o['prune'] = o.get('prune', True)
        o['cache'] = o.get('cache', False)
        o['debug_pruning'] = o.get('debug_pruning', False)

    def start(self):
        o = self.options
        n0 = Node.next_node_number
        t0 = time.time()
        g = Greiner(self.description, self.handle, o)
        self.handle = g.hsdag()
        n1 = Node.next_node_number
        hs = hittingsets_from_tree(self.handle, (o['min_card'], o['max_card']))
        t1 = time.time()
        d = self.description
        self.result = Result(hs, total_time=t1 - t0, tp_time_check=d.get_check_time(), tp_time_comp=d.get_comp_time(),
                             num_tpcalls_check=d.get_check_calls(),
                             num_tpcalls_comp=d.get_comp_calls(), num_nodes=len(self.handle.nodes()),
                             num_gen_nodes=n1 - n0, cache_hits=g.cache_hits,
                             cache_misses=g.cache_misses, cache_size=len(self.handle.cs_cache), time_map=g.time_map,
                             timeout=g.timeout)


class PyHsDagOEngine(PyHsDagEngine):
    def start(self):
        o = self.options
        n0 = Node.next_node_number
        t0 = time.time()
        g = GreinerO(self.description, self.handle, o)
        self.handle = g.hsdag()
        n1 = Node.next_node_number
        hs = hittingsets_from_tree(self.handle, (o['min_card'], o['max_card']))
        t1 = time.time()
        d = self.description
        self.result = Result(hs, total_time=t1 - t0, tp_time_check=d.get_check_time(), tp_time_comp=d.get_comp_time(),
                             num_tpcalls_check=d.get_check_calls(),
                             num_tpcalls_comp=d.get_comp_calls(), num_nodes=len(self.handle.nodes()),
                             num_gen_nodes=n1 - n0, cache_hits=g.cache_hits,
                             cache_misses=g.cache_misses, cache_size=len(self.handle.cs_cache), time_map=g.time_map,
                             timeout=g.timeout)


class PyHSTEngine(Engine):
    def __init__(self, description, options):
        super(PyHSTEngine, self).__init__(description, options)
        o = self.options
        o['min_card'] = o.get('min_card', 1)
        o['max_card'] = o.get('max_card', sys.maxint)
        o['max_time'] = o.get('max_time', None)
        o['prune'] = o.get('prune', True)
        o['cache'] = o.get('cache', False)

    def start(self):
        o = self.options
        n0 = HSTNode.next_node_number
        t0 = time.time()
        h = HST(self.description, self.handle, o)
        self.handle = h.hst()
        hs = hittingsets_from_tree(self.handle, (o['min_card'], o['max_card']))
        n1 = HSTNode.next_node_number
        t1 = time.time()
        d = self.description
        self.result = Result(hs, total_time=t1 - t0, tp_time_check=d.get_check_time(), tp_time_comp=d.get_comp_time(),
                             num_tpcalls_check=d.get_check_calls(),
                             num_tpcalls_comp=d.get_comp_calls(), num_nodes=self.handle.num_nodes(),
                             num_gen_nodes=n1 - n0, cache_hits=h.cache_hits,
                             cache_misses=h.cache_misses, cache_size=len(self.handle.cs_cache))


class PyHSTOEngine(PyHSTEngine):
    def start(self):
        o = self.options
        n0 = HSTNode.next_node_number
        t0 = time.time()
        h = HSTO(self.description, self.handle, o)
        self.handle = h.hst()
        hs = hittingsets_from_tree(self.handle, (o['min_card'], o['max_card']))
        n1 = HSTNode.next_node_number
        t1 = time.time()
        d = self.description
        self.result = Result(hs, total_time=t1 - t0, tp_time_check=d.get_check_time(), tp_time_comp=d.get_comp_time(),
                             num_tpcalls_check=d.get_check_calls(),
                             num_tpcalls_comp=d.get_comp_calls(), num_nodes=self.handle.num_nodes(),
                             num_gen_nodes=n1 - n0, cache_hits=h.cache_hits,
                             cache_misses=h.cache_misses, cache_size=len(self.handle.cs_cache))


class PyHsDagNEngine(PyHsDagEngine):
    def start(self):
        o = self.options
        n0 = Node.next_node_number
        t0 = time.time()
        g = GreinerN(self.description, self.handle, o)
        self.handle = g.hsdag()
        n1 = Node.next_node_number
        hs = hittingsets_from_tree(self.handle, (o['min_card'], o['max_card']))
        t1 = time.time()
        d = self.description
        self.result = Result(hs, total_time=t1 - t0, tp_time_check=d.get_check_time(), tp_time_comp=d.get_comp_time(),
                             num_tpcalls_check=d.get_check_calls(),
                             num_tpcalls_comp=d.get_comp_calls(), num_nodes=len(self.handle.nodes()),
                             num_gen_nodes=n1 - n0, cache_hits=g.cache_hits,
                             cache_misses=g.cache_misses, cache_size=len(self.handle.cs_cache), time_map=g.time_map,
                             timeout=g.timeout)
