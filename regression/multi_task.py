import os
import time
import sys
import getopt
from multiprocessing import Process, Pool
import regression_config as rcfg
import subprocess

def run_case(name, simulator, cfg):
  print('Run Case: %s' % name)
  vcs = 'make simv-run'
  emu = 'make emu_rtl-run'
  simulation_prefix = cfg['config']['case_prefix']
  shell = ''
  if simulator == 'vcs':
    shell = vcs + " RUN_BIN=" + simulation_prefix + name
  elif simulator == 'verilator':
    shell = emu + " RUN_BIN=" + simulation_prefix + name
  print(shell)
  os.system(shell)

def run_case_timeout(name, simulator, cfg):
  print('Run Case: %s' % name)
  simulation_prefix = cfg['config']['case_prefix']
  comp_dir = os.path.abspath('./') + '/sim/rtl/comp'
  simv_dir = os.path.abspath('./') + '/sim/rtl/' + simulation_prefix + name
  workload_dir = cfg['config']['workload']
  create_simdir = 'mkdir -p ' + simv_dir
  creat_simlog = 'touch ' + simv_dir + '/sim.log'
  command_ln1 = 'ln -s ' + comp_dir + '/simv ' + simv_dir + '/simv'
  command_ln2 = 'ln -s ' + comp_dir + '/simv.daidir ' + simv_dir + '/simv.daidir'
  print(creat_simlog)
  print(command_ln1)
  print(command_ln2)
  os.system(create_simdir + '&&' + creat_simlog + '&&' + command_ln1 + '&&' + command_ln2)
  run_work_load = '+workload=' + workload_dir + simulation_prefix + name
  run_diff = '+diff=' + workload_dir + 'riscv64-spike-so'
  run_opts = '-fgp=num_threads:4,num_fsdb_threads:4,allow_less_cores'
  run_assert = ['-assert', 'finish_maxfail=30', '-assert', 'global_finish_maxfail=10000', '2>', 'assert.log', '|', 'tee', 'sim.log']
  try:
    run_proc = subprocess.run(['./simv', run_work_load, run_diff, run_opts] + run_assert, cwd=simv_dir, timeout=10)
  except subprocess.TimeoutExpired:
    run_proc.kill()
    run_proc.wait()

def multi_task_run(task_num, cfg, case_file_list):
  simulator = cfg['config']['simulator']

  test_case_list = rcfg.get_case_list('./regression', case_file_list)
  print(test_case_list)

  pool = Pool(task_num)

  for list_name in test_case_list.keys():
    case_list = test_case_list[list_name]
    for case in case_list:
      pool.apply_async(func = run_case_timeout, args = (case, simulator, cfg))
  pool.close()
  pool.join()