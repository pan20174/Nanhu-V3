import os
import toml
import time
import sys
from pprint import pprint

def case_list_check(all_list, part_list):
  for part in part_list:
    if part not in all_list:
      return False
  return True

def parse_config(path):
  cfg_file = os.path.abspath(path + "/regression.toml")
  cfg = toml.load(cfg_file)
  pprint(cfg)

  run_config = cfg['config']
  all_list = cfg['case']['list']

  simulator = ''
  test_list = []
  if run_config['simulator'] == 'verilator':
    simulator = 'verilator'
  else:
    simulator = 'vcs'
  
  if run_config['sim_mode'] == 'part':
    test_list = run_config['part_list']
  else:
    test_list = cfg['case']['list']
  
  if case_list_check(all_list, test_list):
    print("\n******The following use cases will begin testing:  ******")
    for case in test_list:
      print("* test case: %s" % case)
    print("*********************************************************\n")
    time.sleep(3)
  else:
    print("test case list illegal, please check it!")
    sys.exit()

  return cfg

def get_case_list(path, test_list):
  case_list = {}
  file_list = []
  for test_name in test_list:
    file_name = path + "/case/" + test_name + ".txt"
    with open(file_name, 'r') as f:
      for line in f:
        if(test_name not in case_list.keys()):
          case_list[test_name] = []
        line = line.replace('\n', '').replace('\r', '')
        case_list[test_name].append(line)
  return case_list