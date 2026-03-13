#!/usr/bin/env python
# encoding: utf-8

from waflib.Utils import to_list
from waflib.TaskGen import before_method, feature
import os
import pickle
import re


_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/rp2xxxChibiOS')
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
        OPTIMIZE = "-O2"
    env.CFLAGS += [ OPTIMIZE ]
    env.CXXFLAGS += [ OPTIMIZE ]
    if env.DEBUG or env.DEBUG_SYMBOLS:
        env.CHIBIOS_BUILD_FLAGS += ' USE_OPT="-g3 -gdwarf-4 -falign-functions=16 -fno-omit-frame-pointer"'
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


def setup_default_parameters(cfg, env, srcpath):
    '''embed board-local or explicitly requested default parameters via ROMFS'''
    defaults_path = None
    if cfg.options.default_parameters:
        defaults_path = cfg.options.default_parameters
        cfg.msg('Default parameters', defaults_path, color='YELLOW')
    else:
        board_defaults = srcpath('libraries/AP_HAL_rp2xxxChibiOS/hwdef/%s/defaults.parm' % env.BOARD)
        if os.path.exists(board_defaults):
            defaults_path = board_defaults
            cfg.msg('Default parameters', board_defaults, color='YELLOW')

    if not defaults_path:
        return

    env.DEFAULT_PARAMETERS = defaults_path
    env.ROMFS_FILES += [('defaults.parm', defaults_path)]
    env.DEFINES += ['HAL_PARAM_DEFAULTS_PATH="@ROMFS/defaults.parm"']


def configure(cfg):
    cfg.find_program("make", var="MAKE") # GNU make is required
    # ChibiOS config
    configureChibiOS(cfg)


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

    env.CH_ROOT = srcpath('modules/rp2xxxChibiOS')
    # env.CC_ROOT = srcpath('modules/CrashDebug/CrashCatcher')
    env.AP_HAL_ROOT = srcpath('libraries/AP_HAL_rp2xxxChibiOS')
    env.BUILDDIR = bldpath('modules/rp2xxxChibiOS')
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.MKFW_TOOLS = srcpath('Tools/ardupilotwaf')

    # relative paths to pass to make, relative to directory that make is run from
    env.CH_ROOT_REL = os.path.relpath(env.CH_ROOT, env.BUILDROOT)
    # env.CC_ROOT_REL = os.path.relpath(env.CC_ROOT, env.BUILDROOT)
    env.AP_HAL_REL = os.path.relpath(env.AP_HAL_ROOT, env.BUILDROOT)
    env.BUILDDIR_REL = os.path.relpath(env.BUILDDIR, env.BUILDROOT)

    mk_custom = srcpath('libraries/AP_HAL_rp2xxxChibiOS/hwdef/%s/chibios_board.mk' % env.BOARD)
    mk_common = srcpath('libraries/AP_HAL_rp2xxxChibiOS/hwdef/common/chibios_board.mk')
    # see if there is a board specific make file
    if os.path.exists(mk_custom):
        env.BOARD_MK = mk_custom
    else:
        env.BOARD_MK = mk_common

    setup_default_parameters(cfg, env, srcpath)

    env.CHIBIOS_BUILD_FLAGS = ''
    load_env_vars(cfg.env)
    setup_optimization(cfg.env)

def pre_build(bld):
    '''pre-build hook to change dynamic sources'''
    setup_optimization(bld.env)


def build(bld):
    # build ChibiOS
    buildChibiOS(bld)

    if bld.env.RP_MCU == 'rp2040':
        # RP2040 stage-2 bootloader object
        bld.env.LDFLAGS += [
                os.path.join(bld.env.BUILDDIR, 'obj', 'bs2_default_padded_checksummed.o')
        ]
    elif bld.env.RP_MCU == 'rp2350':
        # RP2350 PICOBIN image definition block (bootrom searches for this to boot)
        bld.env.LDFLAGS += [
                os.path.join(bld.env.BUILDDIR, 'obj', 'rp2350_imagedef.o')
        ]


def buildChibiOS(bld):
    bld(
        # create the file modules/ChibiOS/include_dirs
        rule="touch Makefile && BUILDDIR=${BUILDDIR_REL} CHIBIOS=${CH_ROOT_REL} " + 
            "AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${MAKE} pass -f '${BOARD_MK}'",
        group='dynamic_sources',
        target=bld.bldnode.find_or_declare('modules/rp2xxxChibiOS/include_dirs')
    )

    common_src = [
        bld.bldnode.find_or_declare('modules/rp2xxxChibiOS/include_dirs')
    ]
    common_src += bld.path.ant_glob('modules/rp2xxxChibiOS/os/hal/**/*.[ch]')
    common_src += bld.path.ant_glob('modules/rp2xxxChibiOS/os/hal/**/*.mk')

    if bld.env.ENABLE_CRASHDUMP:
        # TODO(szilveszter)
        pass
    else:
        ch_task = bld(
            # build libch.a from ChibiOS sources and hwdef.h
            rule="BUILDDIR='${BUILDDIR_REL}' CHIBIOS='${CH_ROOT_REL}' " + 
                "AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} " +
                "'${MAKE}' -j%u lib -f '${BOARD_MK}'" % bld.options.jobs,
            group='dynamic_sources',
            source=common_src,
            target=bld.bldnode.find_or_declare('modules/rp2xxxChibiOS/libch.a')
        )

    
    ch_task.name = "ChibiOS_lib"
    bld.env.LIB += ['ch']
    bld.env.LIBPATH += ['modules/rp2xxxChibiOS/']
    if bld.env.ENABLE_CRASHDUMP:
        bld.env.LINKFLAGS += ['-Wl,-whole-archive', 'modules/rp2xxxChibiOS/libcc.a', '-Wl,-no-whole-archive']
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

    if bld.env.RP_MCU == 'rp2040':
        # RP2040 ROM float shim: redirect soft-float calls to ROM-backed routines
        float_wraplist = [
            '__aeabi_frsub', '__aeabi_fsub', '__aeabi_fadd', '__aeabi_fdiv', '__aeabi_fmul',
            '__aeabi_cfrcmple', '__aeabi_cfcmple', '__aeabi_cfcmpeq',
            '__aeabi_fcmpeq', '__aeabi_fcmplt', '__aeabi_fcmple',
            '__aeabi_fcmpge', '__aeabi_fcmpgt', '__aeabi_fcmpun',
            '__aeabi_ui2f', '__aeabi_i2f', '__aeabi_f2iz', '__aeabi_f2uiz',
            '__aeabi_l2f', '__aeabi_ul2f', '__aeabi_f2lz', '__aeabi_f2ulz',
            '__aeabi_f2d',
            'sqrtf', 'cosf', 'sinf', 'sincosf', 'tanf', 'atan2f', 'expf', 'logf',
        ]
        for w in float_wraplist:
            bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]

        # RP2040 ROM double shim: redirect soft-double calls to ROM-backed routines
        double_wraplist = [
            '__aeabi_drsub', '__aeabi_dsub', '__aeabi_dadd', '__aeabi_ddiv', '__aeabi_dmul',
            '__aeabi_cdrcmple', '__aeabi_cdcmple', '__aeabi_cdcmpeq',
            '__aeabi_dcmpeq', '__aeabi_dcmplt', '__aeabi_dcmple',
            '__aeabi_dcmpge', '__aeabi_dcmpgt', '__aeabi_dcmpun',
            '__aeabi_ui2d', '__aeabi_i2d', '__aeabi_d2iz', '__aeabi_d2uiz',
            '__aeabi_l2d', '__aeabi_ul2d', '__aeabi_d2lz', '__aeabi_d2ulz',
            '__aeabi_d2f',
            'sqrt', 'cos', 'sin', 'tan', 'atan2', 'exp', 'log',
        ]
        for w in double_wraplist:
            bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]
    elif bld.env.RP_MCU == 'rp2350':
        # RP2350 DCP double shim: redirect double-precision calls to pico-sdk
        # wrappers that use the hardware double co-processor where applicable.
        double_wraplist = [
            '__aeabi_drsub', '__aeabi_dsub', '__aeabi_dadd', '__aeabi_ddiv', '__aeabi_dmul',
            '__aeabi_cdrcmple', '__aeabi_cdcmple', '__aeabi_cdcmpeq',
            '__aeabi_dcmpeq', '__aeabi_dcmplt', '__aeabi_dcmple',
            '__aeabi_dcmpge', '__aeabi_dcmpgt', '__aeabi_dcmpun',
            '__aeabi_ui2d', '__aeabi_i2d', '__aeabi_d2iz', '__aeabi_d2uiz',
            '__aeabi_l2d', '__aeabi_ul2d', '__aeabi_d2lz', '__aeabi_d2ulz',
            '__aeabi_d2f',
            'sqrt', 'cos', 'sin', 'tan', 'atan2', 'exp', 'log',
            'ldexp', 'copysign', 'trunc', 'floor', 'ceil', 'round',
            'sincos', 'asin', 'acos', 'atan', 'sinh', 'cosh', 'tanh',
            'asinh', 'acosh', 'atanh', 'exp2', 'log2', 'exp10', 'log10',
            'pow', 'powint', 'hypot', 'cbrt', 'fmod', 'drem', 'remainder',
            'remquo', 'expm1', 'log1p', 'fma',
        ]
        for w in double_wraplist:
            bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]
