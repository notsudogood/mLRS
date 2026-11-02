#!/usr/bin/env python
'''
*******************************************************
 Swarm C2 TDMA firmware
 GPL3
*******************************************************
 run_make_tdma.py
 Builds the TDMA firmware (mLRS/tdma/) for the Matek mR900-30.
 Reuses the mLRS hardware layer and the compile/link steps of run_make_firmwares.py,
 but compiles only the HAL, the SX126x driver, the USB CDC device and the tdma sources:
 no mLRS link logic.
 Core (startup, clocks, ST drivers incl. USB device library) comes from the mLRS TX target
 tx-matek-mr900-30-g431kb, which is the same board with its USB-C port enabled. The pin
 hal is the RX one (rx-hal-matek-mr900-30-g431kb.h), so TX1/RX1 stay UART1.

 usage:
   python3 run_make_tdma.py --role ground
   python3 run_make_tdma.py --role drone --node 1
   python3 run_make_tdma.py --role drone --node 1 --power 24
   python3 run_make_tdma.py --all          (ground + drones 1, 2, 3)

 output: tools/build/firmware/tdma-<role>-n<node>-<hash>.hex and .bin (for dfu-util)
 Run run_setup.py (or at least run_copy_st_drivers.py) once beforehand.
********************************************************
'''
import os
import sys
import shutil
import argparse
import subprocess
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_make_firmwares as mk


TARGET = 'tx-matek-mr900-30-g431kb'     # Core dir: same board, USB-C enabled
TARGET_D_CORE = 'TX_MATEK_MR900_30_G431KB' # only to satisfy cTarget's rx/tx name check
TARGET_D = 'RX_MATEK_MR900_30_G431KB'      # selects the pin hal for the .cpp files

TDMA_SOURCES_MODULES = [
    os.path.join('modules','sx12xx-lib','src','sx126x.cpp'),
    os.path.join('modules','stm32ll-lib','src','stdstm32.c'),
    ]

TDMA_SOURCES_COMMON = [
    os.path.join('Common','libs','utils.cpp'),
    ]

TDMA_SOURCES = [
    os.path.join('tdma','tdma-main.cpp'),
    ]

POWER_MAX_DBM = 24 # bench cap, also enforced in tdma/config.h and tdma/radio.h


def find_gcc_dir():
    gcc = shutil.which('arm-none-eabi-gcc')
    if not gcc:
        print('ERROR: arm-none-eabi-gcc not found on PATH')
        sys.exit(1)
    return os.path.dirname(gcc)


def git_hash():
    try:
        return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=mk.MLRS_PROJECT_DIR, text=True).strip()
    except Exception:
        return 'nogit'


def build(role, node, power):
    name = 'tdma-' + role + '-n' + str(node)
    defines = [
        'STDSTM32_USE_USB',
        'TDMA_ROLE=TDMA_ROLE_' + role.upper(),
        'TDMA_NODE_ID=' + str(node),
        ]
    if power is not None:
        defines.append('TDMA_TX_POWER_DBM=' + str(power))
        name += '-p' + str(power)

    target = mk.cTargetG431KB(TARGET, TARGET_D_CORE, defines, name, name + '-' + git_hash(), '')
    target.target_D = TARGET_D

    print('------------------------------------------------------------')
    print('target', name, defines)

    mk.create_clean_dir(os.path.join(mk.MLRS_BUILD_DIR, target.build_dir))

    files = [os.path.join(target.target, 'Core', 'Startup', target.startup_script)]
    files += [os.path.join(target.target, f) for f in target.MLRS_SOURCES_HAL]
    files += [os.path.join(target.target, f) for f in target.MLRS_SOURCES_CORE]
    files += [os.path.join(target.target, f) for f in target.MLRS_SOURCES_EXTRA] # USB device
    files += TDMA_SOURCES_MODULES + TDMA_SOURCES_COMMON + TDMA_SOURCES

    for f in files:
        mk.create_dir(os.path.join(mk.MLRS_BUILD_DIR, target.build_dir, os.path.dirname(f)))
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as pool:
        list(pool.map(lambda f: mk.mlrs_compile_file(target, f), files))

    print('linking')
    mk.mlrs_link_target(target)

    elf = os.path.join(mk.MLRS_BUILD_DIR, target.build_dir, target.elf_name + '.elf')
    if not os.path.exists(elf):
        print('ERROR: build failed for', name)
        return False
    os.system(os.path.join(mk.GCC_DIR, 'arm-none-eabi-size') + ' "' + elf + '"')

    hexfile = os.path.join(mk.MLRS_BUILD_DIR, 'firmware', target.elf_name + '.hex')
    mk.create_dir(os.path.dirname(hexfile))
    os.system(os.path.join(mk.GCC_DIR, 'arm-none-eabi-objcopy') + ' -O ihex "' + elf + '" "' + hexfile + '"')
    binfile = os.path.splitext(hexfile)[0] + '.bin'
    os.system(os.path.join(mk.GCC_DIR, 'arm-none-eabi-objcopy') + ' -O binary "' + elf + '" "' + binfile + '"')
    print('hex:', hexfile)
    print('bin:', binfile, '(dfu-util -a 0 -s 0x08000000:leave -D <bin>)')
    return True


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description='build the Swarm C2 TDMA firmware for the Matek mR900-30')
    ap.add_argument('--role', choices=['ground', 'drone'])
    ap.add_argument('--node', type=int, default=None, help='node ID, 0 for ground, 1..20 for drones')
    ap.add_argument('--power', type=int, default=None, help='TX power in dBm (default 17, max %d)' % POWER_MAX_DBM)
    ap.add_argument('--all', action='store_true', help='build ground + drones 1, 2, 3')
    args = ap.parse_args()

    if args.power is not None and args.power > POWER_MAX_DBM:
        print('ERROR: --power above the %d dBm bench cap' % POWER_MAX_DBM)
        sys.exit(1)

    mk.GCC_DIR = find_gcc_dir()

    if args.all:
        jobs = [('ground', 0), ('drone', 1), ('drone', 2), ('drone', 3)]
    elif args.role == 'ground':
        jobs = [('ground', 0 if args.node is None else args.node)]
    elif args.role == 'drone':
        if args.node is None:
            print('ERROR: --node is required for a drone node')
            sys.exit(1)
        jobs = [('drone', args.node)]
    else:
        ap.print_help()
        sys.exit(1)

    ok = all([build(role, node, args.power) for role, node in jobs])
    sys.exit(0 if ok else 1)
