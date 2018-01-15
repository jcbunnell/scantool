###################################################################
# Build Options:
#   mode=debug,release  specify either debug or release build.
#   os=wxp,wlh,win7     specify OS version to build for
#                       wxp for Windows XP, WLH for Windows Vista
#                       and win7 for Windows 7.
#   arch=x86,amd64      Specify either X86 or AMD64 platform build.
#
# Build Outputs:
#   Build outputs are located within each module under mode\os\arch
#
# Supported scons builds:
#   scons mode=debug os=win7 arch=x86   (Win7 X86 debug build)
#   scons mode=debug os=win7 arch=amd64 (Win7 AMD64 debug build)
#
#   scons mode=release os=win7 arch=x86   (Win7 X86 release build)
#   scons mode=release os=win7 arch=amd64 (Win7 AMD64 release build)
###################################################################
import os
import sys
import SCons.Util

def generate_flags(mode, arch, os):
    # Common user mode cflags
    cflags = SCons.Util.CLVar("/W4 /GS- /EHsc /Zl /Zc:wchar_t-")

    # Common user mode lflags
    if arch == BUILD_TARGET_AMD64:
        lflags = SCons.Util.CLVar("/MACHINE:X64 /ignore:4254 /NODEFAULTLIB")
    else:
        lflags = SCons.Util.CLVar("/MACHINE:X86 /ignore:4254 /NODEFAULTLIB")

#    if '-s' in set(sys.argv):
    if True:
        cflags += SCons.Util.CLVar("/nologo")
        lflags += SCons.Util.CLVar("/nologo")

    if mode == BUILD_MODE_RELEASE:
        cflags += SCons.Util.CLVar("/O1 /Os /Oi /Ob2 /Gy /DNDEBUG /MD")
        lflags += SCons.Util.CLVar("/INCREMENTAL:NO")
        lflags += SCons.Util.CLVar("/OPT:REF,ICF /RELEASE")
    elif mode == BUILD_MODE_DEBUG:
        cflags += SCons.Util.CLVar("/Od /D_DEBUG /DAC_ENABLE_DEBUG /MD /Z7")
        lflags += SCons.Util.CLVar("/DEBUG")
    else:  # what to do with all?
        print ("Mode ERROR: unknown mode:", mode)
        Exit(1)

    return (cflags, lflags)



EnsureSConsVersion(2,0)
mymode = ARGUMENTS.get("mode", BUILD_MODE_DEBUG)
myarch = ARGUMENTS.get("arch", BUILD_TARGET_AMD64)
myos   = ARGUMENTS.get("os",   BUILD_OS_TARGET_WIN_7)
mykey  = ARGUMENTS.get("key","")
if mymode not in BUILD_MODE_OPTIONS:
    print ('Mode ERROR: found "%s", expecting an entry from:' % mymode, BUILD_MODE_OPTIONS)
    Exit(1)
if myarch not in BUILD_TARGET_OPTIONS:
    print ('Arch ERROR: found "%s", expecting an entry from:' % myarch, BUILD_TARGET_OPTIONS)
    Exit(1)
if myos not in BUILD_OS_TARGET_OPTIONS:
    print ('OS Error: found "%s", expecting an entry from' % myos, BUILD_OS_TARGET_OPTIONS)
    Exit(1)

try:
    # win32api is part of pywin32, which is required, on windows, for proper
    # file locking when running multiple jobs.
    import win32api
except ImportError:
    print ("pywin32 not detected. Install it to enable multiple jobs.")
#else:
    #SetOption('num_jobs', 4)

# By default SCons will calculate the MD5 checksum of every source file in your
# build each time it is run, and will only cache the checksum after the file is
# 2 days old. This default of 2 days is to protect from clock skew from NFS or
# revision control systems. You can tweak this delay using max-drift=SECONDS
# where SECONDS is some number of seconds. Decreasing SECONDS can improve build
# speed by eliminating superfluous MD5 checksum calculations.
#SetOption('max_drift', 1)
#SetOption('implicit_cache', 1)

env = Environment(tools = [],
                  DDK_OS_TARGET = None,
                  USER_CCFLAGS_COMMON = None,
                  USER_LINKFLAGS_COMMON = None,
                  WDK_VERSION = '7600.16385.1',
                  TARGET_ARCH = None,
                  USE_STL = True,
                  STL_VER = 70)


"""
Because we're building with the DDK, we're linking against the standard msvcrt.
The compiler, however, adds additional stuff that is typically in msvcrt80.dll.
We must link in a precompiled object to include this extra code.  This 'crt'
needs to be linked into each EXE and DLL
"""

if mymode == BUILD_MODE_RELEASE:
    env['OACR_ENABLE'] = True
else:
    env['OACR_ENABLE'] = False

crt = None #winddk + '/lib/wxp/i386/msvcrt_winxp.obj'
env['ENV']['crt'] = crt
env['ENV']['INCLUDE'] = "."
env['ENV']['LIB'] = ""
env['MODE'] = mymode
env["LIB_PATHS"] = {}
env["INC_PATHS"] = {}
env["OACR_USER_INI_DIR"] = env.Dir("#").get_abspath()

env['KERNEL_BUILD_OPTIONS'] = [
]

env['USER_BUILD_OPTIONS'] = [
]

#CacheDir('#build_cache')

environments = construct_environments(env, mymode, myarch, myos, generate_flags)

for e in environments:

    SConscript(
        "SConscript",
        variant_dir = "#" + e["OUTFOLDER"],
        exports = {"env": e});

# Remove all build directories, type 'scons distclean' on command line
env.Command("distclean", [], cleanBuildDirs)
