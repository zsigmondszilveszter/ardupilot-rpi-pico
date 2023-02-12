#!/usr/bin/env python
# encoding: utf-8

from waflib.Utils import to_list
from waflib.TaskGen import before_method, feature
import os
import pickle
import re


_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/rp2040ChibiOS')
    tmp_str = bldnode.find_node('include_dirs').read()
    tmp_str = tmp_str.replace(';\n','')
    tmp_str = tmp_str.replace('-I','')  #remove existing -I flags
    # split, coping with separator
    idirs = re.split('; ', tmp_str)

    # create unique list, coping with relative paths
    idirs2 = []
    for d in idirs:
        if d.startswith('../'):
            # relative paths from the make build are relative to BUILDROOT
            d = os.path.join(bld.env.BUILDROOT, d)
        d = os.path.normpath(d)
        if not d in idirs2:
            idirs2.append(d)
    _dynamic_env_data['include_dirs'] = idirs2

@feature('ch_ap_library', 'ch_ap_program')
@before_method('process_source')
def ch_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)
    self.use += ' ch'
    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])

def setup_optimization(env):
    '''setup optimization flags for build'''
    if env.DEBUG:
        OPTIMIZE = "-Og"
    elif env.OPTIMIZE:
        OPTIMIZE = env.OPTIMIZE
    else:
        OPTIMIZE = "-Os"
    env.CFLAGS += [ OPTIMIZE ]
    env.CXXFLAGS += [ OPTIMIZE ]
    env.CHIBIOS_BUILD_FLAGS += ' USE_COPT=%s' % OPTIMIZE

def load_env_vars(env):
    '''optionally load extra environment variables from env.py in the build directory'''
    print("Checking for env.py")
    env_py = os.path.join(env.BUILDROOT, 'env.py')
    if not os.path.exists(env_py):
        print("No env.py found")
        return
    e = pickle.load(open(env_py, 'rb'))
    for k in e.keys():
        v = e[k]
        if k == 'ROMFS_FILES':
            env.ROMFS_FILES += v
            continue
        if k in env:
            if isinstance(env[k], dict):
                a = v.split('=')
                env[k][a[0]] = '='.join(a[1:])
                print("env updated %s=%s" % (k, v))
            elif isinstance(env[k], list):
                env[k].append(v)
                print("env appended %s=%s" % (k, v))
            else:
                env[k] = v
                print("env added %s=%s" % (k, v))
        else:
            env[k] = v
            print("env set %s=%s" % (k, v))
    if env.ENABLE_ASSERTS:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_ASSERTS=yes'
    if env.ENABLE_MALLOC_GUARD:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_MALLOC_GUARD=yes'
    if env.ENABLE_STATS:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_STATS=yes'
    if env.ENABLE_DFU_BOOT and env.BOOTLOADER:
        env.CHIBIOS_BUILD_FLAGS += ' USE_ASXOPT=-DCRT0_ENTRY_HOOK=TRUE'


def configure(cfg):
    cfg.find_program("make", var="MAKE") # GNU make is required
    # ChibiOS config
    configureChibiOS(cfg)
    # define boot_stage2 location
    cfg.env.PICO_BOOT_STAGE2 = cfg.env.AP_HAL_ROOT+'/hwdef/rp2040-pico/bs2_default_padded_checksummed.S'


def configureChibiOS(cfg): 
    cfg.find_program('arm-none-eabi-objcopy', var='OBJCOPY')
    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.AP_PROGRAM_FEATURES += ['ch_ap_program']

    kw = env.AP_LIBRARIES_OBJECTS_KW
    kw['features'] = to_list(kw.get('features', [])) + ['ch_ap_library']

    env.CH_ROOT = srcpath('modules/rp2040ChibiOS')
    # env.CC_ROOT = srcpath('modules/CrashDebug/CrashCatcher')
    env.CH_CONTRIB_ROOT = srcpath('modules/ChibiOS-Contrib')
    env.AP_HAL_ROOT = srcpath('libraries/AP_HAL_rp2040ChibiOS')
    env.BUILDDIR = bldpath('modules/rp2040ChibiOS')
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.MKFW_TOOLS = srcpath('Tools/ardupilotwaf')

    # relative paths to pass to make, relative to directory that make is run from
    env.CH_ROOT_REL = os.path.relpath(env.CH_ROOT, env.BUILDROOT)
    env.CH_CONTRIB_ROOT_REL = os.path.relpath(env.CH_CONTRIB_ROOT, env.BUILDROOT)
    # env.CC_ROOT_REL = os.path.relpath(env.CC_ROOT, env.BUILDROOT)
    env.AP_HAL_REL = os.path.relpath(env.AP_HAL_ROOT, env.BUILDROOT)
    env.BUILDDIR_REL = os.path.relpath(env.BUILDDIR, env.BUILDROOT)

    mk_custom = srcpath('libraries/AP_HAL_rp2040ChibiOS/hwdef/%s/chibios_board.mk' % env.BOARD)
    mk_common = srcpath('libraries/AP_HAL_rp2040ChibiOS/hwdef/common/chibios_board.mk')
    # see if there is a board specific make file
    if os.path.exists(mk_custom):
        env.BOARD_MK = mk_custom
    else:
        env.BOARD_MK = mk_common

    if cfg.options.default_parameters:
        cfg.msg('Default parameters', cfg.options.default_parameters, color='YELLOW')
        env.DEFAULT_PARAMETERS = cfg.options.default_parameters

    env.CHIBIOS_BUILD_FLAGS = ''
    load_env_vars(cfg.env)
    setup_optimization(cfg.env)

def pre_build(bld):
    '''pre-build hook to change dynamic sources'''
    setup_optimization(bld.env)


def build(bld):
    # build ChibiOS
    buildChibiOS(bld)

    bld.env.LDFLAGS += [
        bld.env.PICO_BOOT_STAGE2
    ]


def buildChibiOS(bld):
    bld(
        # create the file modules/ChibiOS/include_dirs
        rule="touch Makefile && BUILDDIR=${BUILDDIR_REL} CHIBIOS=${CH_ROOT_REL} CHIBIOS_CONTRIB=${CH_CONTRIB_ROOT_REL} " + 
            "AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${MAKE} pass -f '${BOARD_MK}'",
        group='dynamic_sources',
        target=bld.bldnode.find_or_declare('modules/rp2040ChibiOS/include_dirs')
    )

    common_src = [
        bld.bldnode.find_or_declare('modules/rp2040ChibiOS/include_dirs')
    ]
    common_src += bld.path.ant_glob('modules/rp2040ChibiOS/os/hal/**/*.[ch]')
    common_src += bld.path.ant_glob('modules/rp2040ChibiOS/os/hal/**/*.mk')

    if bld.env.ENABLE_CRASHDUMP:
        # TODO(szilveszter)
        pass
    else:
        ch_task = bld(
            # build libch.a from ChibiOS sources and hwdef.h
            rule="BUILDDIR='${BUILDDIR_REL}' CHIBIOS='${CH_ROOT_REL}' CHIBIOS_CONTRIB=${CH_CONTRIB_ROOT_REL} " + 
                "AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} " +
                "'${MAKE}' -j%u lib -f '${BOARD_MK}'" % bld.options.jobs,
            group='dynamic_sources',
            source=common_src,
            target=bld.bldnode.find_or_declare('modules/rp2040ChibiOS/libch.a')
        )

    
    ch_task.name = "ChibiOS_lib"
    bld.env.LIB += ['ch']
    bld.env.LIBPATH += ['modules/rp2040ChibiOS/']
    if bld.env.ENABLE_CRASHDUMP:
        bld.env.LINKFLAGS += ['-Wl,-whole-archive', 'modules/rp2040ChibiOS/libcc.a', '-Wl,-no-whole-archive']
    # list of functions that will be wrapped to move them out of libc into our
    # own code note that we also include functions that we deliberately don't
    # implement anywhere (the FILE* functions). This allows us to get link
    # errors if we accidentially try to use one of those functions either
    # directly or via another libc call
    wraplist = ['sscanf', 'fprintf', 'snprintf', 'vsnprintf','vasprintf','asprintf','vprintf','scanf',
                'fiprintf','printf',
                'fopen', 'fflush', 'fwrite', 'fread', 'fputs', 'fgets',
                'clearerr', 'fseek', 'ferror', 'fclose', 'tmpfile', 'getc', 'ungetc', 'feof',
                'ftell', 'freopen', 'remove', 'vfprintf', 'fscanf',
                '_gettimeofday', '_times', '_times_r', '_gettimeofday_r', 'time', 'clock' ]
    for w in wraplist:
        bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]
