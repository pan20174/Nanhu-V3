import os
import subprocess
import shutil
import argparse
import random
import re
import queue
import concurrent.futures
from tqdm import tqdm
from tabulate import tabulate
import time

PASS = 0
FAILED = 1
TIMEOUT = 2
REDO = 3

class SimWorker:
  caseName = ''
  simDir = ''
  compDir = ''
  cmd = ''
  tool = ''
  timeoutVal = 0
  process = None
  status = PASS
  def __init__(self, case:str, sim:str, comp:str, _tool:str, ref:str, timeoutVal:int, dump:bool):
    self.caseName = os.path.basename(case)
    self.simDir = os.path.join(sim, self.caseName)
    self.compDir = comp
    self.tool = _tool
    self.timeoutVal = timeoutVal
    if(_tool == 'vcs'):
      self.cmd = 'simv'
    else:
      self.cmd = 'emu'
    if os.path.exists(self.simDir):
      shutil.rmtree(self.simDir)
    os.makedirs(self.simDir)
    os.symlink(os.path.join(self.compDir, self.cmd), os.path.join(self.simDir, self.cmd))
    if(_tool == 'vcs'):
      os.symlink(os.path.join(self.compDir, 'simv.daidir'), os.path.join(self.simDir, 'simv.daidir'), True)
      self.cmd += ' +diff=' + ref + ' '
      self.cmd += '-fgp=num_threads:4,num_fsdb_threads:4 '
      self.cmd += '-assert finish_maxfail=30 -assert global_finish_maxfail=10000 '
      self.cmd += '+workload=' + case + ' '
      if(dump):
        self.cmd += '+dump-wave=fsdb '
    else:
      self.cmd += ' -i ' + case + ' '
      self.cmd += '+diff=' + ref + ' '
      self.cmd += '--wave-path ' + os.path.join(self.simDir, self.caseName, 'tb_top.vcd') + ' '
      self.cmd += '--enable-fork --fork-interval=15 '
      self.cmd += '-s ' + random.randint(0, 9999) + ' '
    self.cmd = './' + self.cmd


  def run(self):
    def isIn(sub, full):
      return len(re.findall(sub, full)) != 0

    while True:
      self.process = subprocess.Popen(self.cmd + '2> assert.log | tee sim.log', stdout=subprocess.PIPE, shell=True, cwd=self.simDir)
      tqdm.write('Running: ' + self.caseName)

      try:
        if(self.timeoutVal != 0):
          so, _ = self.process.communicate(timeout=self.timeoutVal)
        else:
          so, _ = self.process.communicate()
      except subprocess.TimeoutExpired:
        self.process.kill()
        so, _ = self.process.communicate()
        self.status = TIMEOUT
        break

      log = so.decode("utf-8")
      if isIn('commit group', log):
        self.status = FAILED
      elif isIn('FGP_AFFINITY_FAILED', log):
        self.status = REDO
      else:
        self.status = PASS

      if self.status != REDO:
        break

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='XiangShan Regression Scripts')
  parser.add_argument('caselist', type=str, help='Case list')
  parser.add_argument('-t', '--tool', default="vcs", type=str, help='simulation tool (Default: vcs)', choices=['vcs', 'verilator'], metavar='')
  parser.add_argument('-r', '--ref', default="ready-to-run/riscv64-spike-so", type=str, help='reference model', metavar='')
  parser.add_argument('-s', '--simdir', default="sim/rtl", type=str, help='simulation directory', metavar='')
  parser.add_argument('-c', '--comp', default="sim/rtl/comp", type=str, help='compile directory', metavar='')
  parser.add_argument('-T', '--timeout', default=1800, type=int, help='case time out value in seconds', metavar='')
  parser.add_argument('-R', '--report', default="regression.rpt", type=str, help='report file', metavar='')
  parser.add_argument('-d', '--cwd', default=".", type=str, help='current work dir', metavar='')
  parser.add_argument('-w', '--wave-enable', action="store_true", default=False, help='enable waveform (Not apllied for verilator)')
  parser.add_argument('-f', '--failed-list', default="failures.txt", type=str, help='failed cases list file', metavar='')
  parser.add_argument('-j', '--jobs', default=8, type=int, help='parallel jobs', metavar='')
  args = parser.parse_args()

  cwd = args.cwd
  if(cwd == '.'):
    cwd = os.path.abspath('.')
  tool = args.tool
  cl = os.path.join(cwd, args.caselist)
  sd = os.path.join(cwd, args.simdir)
  comp = os.path.join(cwd, args.comp)
  to = args.timeout
  ref = os.path.join(cwd, args.ref)
  rpt = os.path.join(cwd, args.report)
  we = args.wave_enable
  ff = args.failed_list
  jobs = args.jobs

  if not os.path.isfile(cl) or not os.path.isfile(ref):
    raise RuntimeError('Illegal file!')

  if not os.path.isdir(sd):
    os.makedirs(sd)

  if not os.path.isdir(comp):
    raise RuntimeError('Illegal compile directory!')

  startTime = time.time()
  caseList = []
  workerQueue = queue.Queue[SimWorker]()
  workerList = []
  with open(cl) as f:
    caseList = f.readlines()
  caseList.sort()
  for c in caseList:
    worker = SimWorker(c.strip('\n'), sd, comp, tool, ref, to, we)
    workerQueue.put(worker)
    workerList.append(worker)

  with concurrent.futures.ThreadPoolExecutor(jobs) as executor:
    results = concurrent.futures.as_completed([executor.submit(lambda x: x.run(), w) for w in workerList])
    list(tqdm(results, total=len(workerList)))

  endTime = time.time()
  timeCostInMin = (endTime - startTime) / 60.0
  print("All Done, Host Time Cost: " + f'{timeCostInMin:.1f}' + " minutes\n")

  workerList.sort(key = lambda x : x.status)
  passNum = 0
  failNum = 0
  timeoutNum = 0
  unknownNum = 0

  caseStatusList = []
  for w in workerList:
    if(w.status == PASS):
      # shutil.rmtree(os.path.join(sd, w.caseName))
      passNum += 1
      caseStatusList.append(w.caseName + ": PASS\n")
    elif(w.status == FAILED):
      failNum += 1
      caseStatusList.append(w.caseName + ": FAILED\n")
    elif(w.status == TIMEOUT):
      timeoutNum += 1
      caseStatusList.append(w.caseName + ": TIMEOUT\n")
    else:
      unknownNum += 1
      caseStatusList.append(w.caseName + ": UNKNOWN\n")

  passRate = passNum * 100.0 / len(caseList)
  failRate = failNum * 100.0 / len(caseList)
  timeoutRate = timeoutNum * 100.0 / len(caseList)
  unknwonRate = unknownNum * 100.0 / len(caseList)
  table = [
    ["", "count", "rate"],
    ["pass", str(passNum), f'{passRate:.1f}%'],
    ["fail", str(failNum), f'{failRate:.1f}%'],
    ["timeout", str(timeoutNum), f'{timeoutRate:.1f}%'],
    ["unknown", str(unknownNum), f'{unknwonRate:.1f}%'],
    ["total", str(len(caseList)), "100.0%"],
  ]
  tableStr = tabulate(table, headers="firstrow", tablefmt="grid")

  with open(rpt, "w") as r:
    r.write("Host Time Cost: " + f'{timeCostInMin:.1f}' + " minutes\n\n")
    r.write(tableStr)
    r.write("\n\n")
    r.write("======================================================================\n\n")
    for l in caseStatusList:
      r.write(l)

  with open(ff, 'w') as f:
    for w in workerList:
      if w.status == FAILED:
        f.write(w.caseName + "\n")
    
  