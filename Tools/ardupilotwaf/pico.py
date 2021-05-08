#!/usr/bin/env python
# encoding: utf-8

from waflib.Configure import conf
from waflib.Utils import check_dir
import subprocess



def options(opt):
    opt.add_option('--PICO_SDK_PATH', action='store', default='', help="The Raspberry Pi Pico SDK location [default: 'modules/pico-sdk']")
    opt.add_option('--PICO_TOOLCHAIN_PATH', action='store', default='/home/szilveszter/toolchains/gcc-arm-none-eabi-10-2020-q4-major', 
                   help="The toolchain location which is used to build the RPI SDK and the code [default: '/home/szilveszter/toolchains/gcc-arm-none-eabi-10-2020-q4-major']")

@conf
def setPicoSDKPath(conf):
    if conf.options.PICO_SDK_PATH:
        conf.env.PICO_SDK_PATH = conf.options.PICO_SDK_PATH
    else:
        conf.env.PICO_SDK_PATH = conf.srcnode.abspath()+'/modules/pico-sdk'
    conf.msg('PICO_SDK_PATH', conf.env.PICO_SDK_PATH)

@conf
def setPicoToolchainPath(conf):
    if conf.options.PICO_TOOLCHAIN_PATH:
        conf.env.PICO_TOOLCHAIN_PATH = conf.options.PICO_TOOLCHAIN_PATH
    conf.msg('PICO_TOOLCHAIN_PATH', conf.env.PICO_TOOLCHAIN_PATH)

@conf
def run_external_cmake_config(conf):
    print('Execute the external Cmake configuration command')
    st = 'cd ' + conf.env.CMAKE_BUILD_DIR_ABS_PATH + ' && env "PICO_SDK_PATH=' + conf.env.PICO_SDK_PATH + '" "PICO_TOOLCHAIN_PATH=' + conf.env.PICO_TOOLCHAIN_PATH +'" ' + conf.env.CMAKE[0] + ' ../../Tools/pico_cmake'
    subprocess.run(st, shell=True)
    conf.msg("Configuring external CMake ", "Done")

@conf
def readFlags(conf):
    with open(conf.env.CMAKE_BUILD_DIR_ABS_PATH+"/CMakeFiles/" + conf.env.CMAKE_MAIN_APP_NAME + ".dir/flags.make") as ins:
        for line in ins:
            line = line.rstrip('\n')
            linecomponents = line.split(" = ", 1)
            if len(linecomponents) > 1:
                env_name = linecomponents[0]
                env_values = linecomponents[1].split(" ")
                conf.env["PICO_"+env_name] = env_values

@conf
def readLinkingStuff(conf):
    with open(conf.env.CMAKE_BUILD_DIR_ABS_PATH+"/CMakeFiles/" + conf.env.CMAKE_MAIN_APP_NAME + ".dir/link.txt") as ins:
        for line in ins:
            line = line.rstrip('\n')
            linecomponents = line.split()
            # remove the compile component from the very beginning of the list
            linecomponents.pop(0)

            pico_link_flags = []
            pico_link_objs = []
            for component in linecomponents:
                if component.startswith("-"):
                    # exception components starting with - we don't want to add them to the linking flags
                    if any(component in s for s in ["-o"]):
                        continue
                    if component == "-Wl,-Map="+conf.env.CMAKE_MAIN_APP_NAME +".elf.map":
                        component = "-Wl,-Map="+conf.env.TARGET_APP_NAME  +".elf.map"
                    pico_link_flags.append(component)
                else:
                    # exclude the cmake main cpp obj file (default it is empty.cpp), the elf file and boot_stage2 file
                    if any(component in s for s in [
                            "CMakeFiles/" + conf.env.CMAKE_MAIN_APP_NAME + ".dir/" +conf.env.CMAKE_MAIN_APP_NAME +".cpp.obj",
                            conf.env.CMAKE_MAIN_APP_NAME+".elf",
                            "pico-sdk/src/rp2_common/boot_stage2/bs2_default_padded_checksummed.S"
                    ]):
                        continue
                    pico_link_objs.append(conf.env.CMAKE_BUILD_DIR+"/"+component)
            conf.env.PICO_LINKFLAGS = pico_link_flags
            conf.env.PICO_LINK_OBJS = pico_link_objs
            # we process only one row, se we break the loop
            break


def configure(conf):
    conf.env.CMAKE_MAIN_APP_NAME = "empty"
    conf.env.TARGET_APP_NAME = "ardupilot_pico"
    # step 1
    conf.setPicoSDKPath()
    conf.setPicoToolchainPath()
    # add the pico tool chain path to system PATH
    conf.environ["PATH"] = conf.env.PICO_TOOLCHAIN_PATH + "/bin:" + conf.environ.get('PATH', '')
    # pico_cmake working directory
    conf.env.CMAKE_BUILD_DIR = 'cmake_build'
    conf.env.CMAKE_BUILD_DIR_ABS_PATH = conf.bldnode.abspath() + '/' + conf.env.CMAKE_BUILD_DIR
    check_dir(conf.env.CMAKE_BUILD_DIR_ABS_PATH)
    
    # step 2
    conf.find_program("cmake", var="CMAKE") # cmake is required
    conf.find_program("make", var="MAKE") # GNU make is required
    # execute cmake config
    conf.run_external_cmake_config()

    # step 3 
    conf.readFlags()

    # step 4
    conf.readLinkingStuff()

    