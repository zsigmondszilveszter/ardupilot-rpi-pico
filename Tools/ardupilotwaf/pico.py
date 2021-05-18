#!/usr/bin/env python
# encoding: utf-8

from waflib.Configure import conf
from waflib.Utils import check_dir
import subprocess
import re


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
    st = 'cd ' + conf.env.CMAKE_BUILD_DIR_ABS_PATH + ' && env "PICO_SDK_PATH=' + conf.env.PICO_SDK_PATH + '" "PICO_TOOLCHAIN_PATH=' + conf.env.PICO_TOOLCHAIN_PATH +'" ' + conf.env.CMAKE[0] + ' ' + conf.env.PICO_SDK_CMAKE_CONFIG
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
                if any(env_name in s for s in ["ASM_DEFINES", "C_DEFINES", "CXX_DEFINES"]):
                    env_values = [e.replace("-D", "") for e in env_values]
                if any(env_name in s for s in ["ASM_INCLUDES", "C_INCLUDES", "CXX_INCLUDES"]):
                    env_values = [e.replace("-I", "") for e in env_values]
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
    conf.env.TARGET_APP_NAME = "ardupilot-rpi-pico"
    # step 1
    conf.setPicoSDKPath()
    conf.setPicoToolchainPath()
    # add the pico tool chain path to system PATH
    conf.environ["PATH"] = conf.env.PICO_TOOLCHAIN_PATH + "/bin:" + conf.environ.get('PATH', '')


def configure2(conf):
    # pico_cmake working directory
    conf.env.CMAKE_BUILD_DIR = 'modules/pico-sdk'
    conf.env.CMAKE_BUILD_DIR_ABS_PATH = conf.bldnode.make_node(conf.variant+'/'+conf.env.CMAKE_BUILD_DIR).abspath()
    check_dir(conf.env.CMAKE_BUILD_DIR_ABS_PATH)
    # pico SDK cmake config files location
    conf.env.PICO_SDK_CMAKE_CONFIG = conf.srcnode.abspath() + "/Tools/pico_cmake"
    
    # step 2
    conf.find_program("cmake", var="CMAKE") # cmake is required
    conf.find_program("make", var="MAKE") # GNU make is required
    # execute cmake config
    conf.run_external_cmake_config()

    # step 3 
    conf.readFlags()

    # step 4
    conf.readLinkingStuff()



def build(bld):
    # add PICO linking flags to waf/ardupilot linking flags
    bld.env.LINKFLAGS += bld.env.PICO_LINKFLAGS

    # define boot_stage2
    bld.env.PICO_BOOT_STAGE2 = bld.env.CMAKE_BUILD_DIR+'/pico-sdk/src/rp2_common/boot_stage2/bs2_default_padded_checksummed.S'

    # build pico's boot_stage2
    bld(
        rule = "cd ${CMAKE_BUILD_DIR_ABS_PATH} && ${MAKE} bs2_default_padded_checksummed_asm", 
        target = bld.env.PICO_BOOT_STAGE2, 
        group = 'dynamic_sources',
        source = "Tools/pico_cmake/CMakeLists.txt"
    )

    # compile the rest of the pico SDK source files
    for bld_target in bld.env.PICO_LINK_OBJS:
        make_target = bld_target[len(bld.env.CMAKE_BUILD_DIR+"/CMakeFiles/"+bld.env.CMAKE_MAIN_APP_NAME+".dir/"):]
        make_target = re.sub('\.(c|cpp|S)\.obj$', '.obj', make_target)
        bld(
            rule = "cd ${CMAKE_BUILD_DIR_ABS_PATH} && ${MAKE} "+make_target, 
            target = bld_target, 
            group = 'dynamic_sources',
            source = "Tools/pico_cmake/CMakeLists.txt"
        )

    # try to build a static library from Pico SDK objects
    bld(
        name = "Pico_sdk_lib",
        rule = "${AR} rcs ${TGT} ${SRC}",
        group = 'dynamic_sources',
        source = bld.env.PICO_LINK_OBJS,
        target = 'lib/libPicoSDK.a'
    )
    #
    bld.env.LDFLAGS += [
        "-Wl,-Bstatic",
        "-Wl,-whole-archive",
        "-lPicoSDK",
        "-Wl,-no-whole-archive",
        bld.env.PICO_BOOT_STAGE2
    ]
