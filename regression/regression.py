import os
import time
import sys
import getopt
from multiprocessing import Process, Pool
import regression_config as rcfg
import multi_task
import record

def main():
	cfg = rcfg.parse_config('./regression')
	test_list = []
	if cfg['config']['sim_mode'] == 'part':
		test_list = cfg['config']['part_list']
	elif cfg['config']['sim_mode'] == 'all':
		test_list = cfg['case']['list']
	print(test_list)
	sim_num = cfg['config']['sim_num']
	#multi_task.multi_task_run(sim_num, cfg, test_list)
	exec_res = record.res_analysis(cfg, test_list)
	print(exec_res)
	record.only_fault_link(cfg, test_list)

if __name__ == '__main__':
	main()