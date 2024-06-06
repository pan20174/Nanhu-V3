from multiprocessing import Pool,Process
import os
import subprocess

build_emu="make -f makefile emu EMU_TRACE=1 EMU_THREADS=16 -j"

case_list = [
    'microbench.bin',
    'linux.bin'
]

def run_case(case):
    cmd="make -f makefile emu_rtl-run RUN_BIN="+case
    print("cmd = " + cmd)
    subprocess.run(cmd,shell=True, check=True)


if __name__ == '__main__':
    # set env
    current_dir = os.getcwd()
    os.environ['NOOP_HOME'] = current_dir

    # build emu
    subprocess.run(build_emu, shell=True, check=True)

    # run case
    process_list = []
    for case_name in case_list:
        p = Process(target=run_case,args=(case_name,))
        p.start()
        process_list.append(p)

    for p in process_list:
        p.join()

    print('All Done!')

