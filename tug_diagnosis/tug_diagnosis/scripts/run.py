#!/usr/bin/env python
from pymbd.diagnosis.problem import Problem
from pymbd.benchmark.tug_description_parser.oracle import TUGDescriptionOracle
from pymbd.util.sethelper import write_sets

p = Problem()

the_list = ['hst-picosat',
            # 'hst-cache-picosat',
            # 'hst-ci-picosat',
            # 'hst-ci-cache-picosat',
            # 'hsdag-picosat',
            # 'hsdag-cache-picosat',
            # 'hsdag-ci-picosat',
            # 'hsdag-ci-cache-picosat',
            # 'hsdag-sicf-picosat',
            # 'hsdag-sicf-cache-picosat'
            ]

for i in the_list:
    o = TUGDescriptionOracle('/home/stefan/ros/diag_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis/scripts/model.sisc', [0, 0])
    r = p.compute_with_description(o, i, max_card=3)
    d = r.get_diagnoses()
    d = map(o.numbers_to_nodes, d)
    print r.get_stats()['total_time'], " ", str(i), " ", write_sets(d)

