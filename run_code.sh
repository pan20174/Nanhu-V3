#!/bin/bash
export NOOP_HOME=$(pwd)
rm build -rf
#make -f makefile-f simv-run RUN_BIN=microbench.bin CONSIDER_FSDB=0
#make -f makefile-f simv-run RUN_BIN=microbench.bin

#make simv -j64
#make simv-run RUN_BIN=microbench.bin
#make simv-run RUN_BIN=linux_spike.bin
#make simv-run RUN_BIN=coremark-3-iteration.bin


#emu + Spike
make -f makefile-f emu EMU_TRACE=1 EMU_THREADS=16 -j
#make -f makefile-f emu_rtl-run RUN_BIN=microbench.bin

make -f makefile-f emu_rtl-run RUN_BIN=linux_spike.bin


date
date > date.log
