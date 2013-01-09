#########################
# Tools Version aliases #
#########################
#to be updated when required

#systemc='systemc-2.3.0'
#systemc='systemc-2.2.1_20100930_beta'
#systemc='systemc-2.3.0_pub_rev_20111121'
#expat='expat-2.0.1'
#lua='lua-5.2.1'
#swig='swig-2.0.4'
#xerces_c='xerces-c-3.1.1'
#mysqlpp='mysql++-3.1.0'
#boost='boost_1_44_0'
#perftools='gperftools-2.0'
#leaktracetool='LeakTrace'
#sds_infra_path='../../../aromana_sds_infra/sds/infra'
#cssi_infra_path='../../../aromana_sds_infra/sds/cssi'
#tlm_vob='../tlm/'
#loki='../loki'

#NOTE: to build with gem5 run fist ../../../tools/scons/bin/scons build/ARM/libgem5_debug.a

#imports

from subprocess import call
import os,platform,sys,SCons.Script,re

# Find Python include and library directories for embedding the
# interpreter.  For consistency, we will use the same Python
# installation used to run scons (and thus this script).  

from distutils import sysconfig

py_getvar = sysconfig.get_config_var

py_debug = getattr(sys, 'pydebug', False)
py_version = 'python' + py_getvar('VERSION') + (py_debug and "_d" or "")

py_general_include = sysconfig.get_python_inc()
py_platform_include = sysconfig.get_python_inc(plat_specific=True)
py_includes = [ py_general_include ]
if py_platform_include != py_general_include:
    py_includes.append(py_platform_include)

py_lib_path = [ py_getvar('LIBDIR') ]
# add the prefix/lib/pythonX.Y/config dir, but only if there is no
# shared library in prefix/lib/.
if not py_getvar('Py_ENABLE_SHARED'):
    py_lib_path.append(py_getvar('LIBPL'))

py_libs = []
for lib in py_getvar('LIBS').split() + py_getvar('SYSLIBS').split():
    assert lib.startswith('-l')
    lib = lib[2:]   
    if lib not in py_libs:
        py_libs.append(lib)
py_libs.append(py_version)


def compareVersions(v1, v2):
    """helper function: compare arrays or strings of version numbers.
    E.g., compare_version((1,3,25), (1,4,1)')
    returns -1, 0, 1 if v1 is <, ==, > v2
    """
    def make_version_list(v):
        if isinstance(v, (list,tuple)):
            return v
        elif isinstance(v, str):
            return map(lambda x: int(re.match('\d+', x).group()), v.split('.'))
        else:
            raise TypeError

    v1 = make_version_list(v1)
    v2 = make_version_list(v2)
    # Compare corresponding elements of lists
    for n1,n2 in zip(v1, v2):
        if n1 < n2: return -1
        if n1 > n2: return  1
    # all corresponding values are equal... see if one has extra values
    if len(v1) < len(v2): return -1
    if len(v1) > len(v2): return  1
    return 0


# Creating the environment

env = Environment(ENV = os.environ)  # Initialize the environment

#get the mode flag from the command line
#default to 'release' if the user didn't specify
build_mode = ARGUMENTS.get('mode', 'release')   #holds current mode

leakcheck=0
gooprof=0
purify=0
insure=0
env_cxx=""
#check if the user has been naughty: only 'debug','release','leakcheck','purify','insure' or 'gooprof' allowed
if not (build_mode in ['debug', 'release','fast']):
   print "Error: expected 'debug','release','fast', found: " + build_mode
   Exit(1)
if sys.platform.startswith("linux"):
    env_cxx=env['CXX']
    print "Detected and using compiler version: "+ env_cxx+'-'+env['CXXVERSION']
elif sys.platform.startswith("win"):
    env_cxx=env['MSVC_VERSION']
    if ((env_cxx == '9.0Exp') or (env_cxx == '9.0')):
        libcxxstring='vc90'
        projectcxxstring='VC9'
    if ((env_cxx == '10.0Exp') or (env_cxx == '10.0')):
        libcxxstring='vc100'
        projectcxxstring='VC10'
    print "Detected and using compiler version: "+ env_cxx

		
binary_width="32"
if (platform.uname()[4] == "x86_64"):
    binary_width="64"
    
if (build_mode == 'debug'):
    build_mode='debug'
    env['BUILD_MODE']='debug'

if (build_mode == 'release'):
    build_mode='release'
    env['BUILD_MODE']='release'

if (build_mode == 'fast'):
    build_mode='fast'
    env['BUILD_MODE']='release'

#building postfix 

if sys.platform.startswith("linux"):
    postfix = platform.uname()[4]+'-'+platform.uname()[0]+'-'+env_cxx+'-'+env['CXXVERSION']+'-'+build_mode
elif sys.platform.startswith("win"):
    postfix = platform.uname()[4]+'-'+platform.uname()[0]+'-'+env_cxx+'-'+build_mode

# selecting files

if sys.platform.startswith("win"):
    if build_mode == 'debug':
        tools_libs = ['systemc','lua51','libboost_graph-'+libcxxstring+'-mt-sgd','libboost_system-'+libcxxstring+'-mt-sgd','libboost_filesystem-'+libcxxstring+'-mt-sgd','xerces-c_3D']
    if build_mode == 'release':
        tools_libs = ['systemc','lua51','libboost_graph-'+libcxxstring+'-mt-s','libboost_system-'+libcxxstring+'-mt-s','libboost_filesystem-'+libcxxstring+'-mt-s','xerces-c_3']
    if build_mode == 'fast':
        tools_libs = ['systemc','lua51','libboost_graph-'+libcxxstring+'-mt-s','libboost_system-'+libcxxstring+'-mt-s','libboost_filesystem-'+libcxxstring+'-mt-s','xerces-c_3']
if sys.platform.startswith("linux"):
    tools_libs = ['systemc','lua','boost_graph','boost_regex','boost_system','boost_filesystem','xerces-c']#,'tcmalloc']

if sys.platform.startswith("win"):
    system_libs=[ 'mysqlclient'] 
if sys.platform.startswith("linux"):
    system_libs=[ 'pthread','stdc++','dl','m','rt','nsl','gcc','c','z'] #,'mysqlclient','ssl','crypto','krb5','k5crypto','kdb5','resolv','z','com_err']
third_part_libs=[]

if (build_mode == 'debug'):
	third_part_libs += ['gzstream','python2.5','elf']
if ((build_mode == 'release') | (build_mode == 'fast')):
	third_part_libs += ['gzstream','python2.5','elf']
	
# setting includes

env.Append(CPPPATH=[ Dir('amba_socket'),Dir('amba_socket'+os.sep+'dependencies'+os.sep+'AMBA-PV'+os.sep+'include'),Dir('gem5'+os.sep+'src'),py_includes,Dir('gem5'+os.sep+'build'+os.sep+'ARM'),Dir('greensocs'+os.sep+'include'),Dir('gem5_ArmA15'+os.sep+'include'),Dir(os.sep+'usr'+os.sep+'local'+os.sep+'systemc-2.3.0'+os.sep+'include') ])

# setting defines

env.Append(CPPDEFINES=['SC_USE_STD_STRING','__SPIRIT_COMPLIANT__','SC_INCLUDE_DYNAMIC_PROCESSES','SYSTEMC','CBA_IF','SC_USE_DYNAMIC_PROCESSES',('DATECODE','now'),('USERID','user'),'GREENSOCS',('HIDE_ME','false'),('BOOST_FILESYSTEM_VERSION','3')])

# cgem defines

#env.MergeFlags({'CPPDEFINES': [ 'PROD_C60','targ_c60sim','TI_SIMULATOR','DONT_USE_IEEE','SIM_RTDX','_PC_','TB_SUPPORT','SIMBRIDGE','NEW_PINC',('RUN_COUNT','25'),
#					 'GEMSIM','SS','CHIP_SIM','ACT_SUPPORT','GEL_DRIVER_STRING_SUPPORT','SYSTEMC_GEM','CGEM_SC','FULL_SYSTEM' ]})
if build_mode == 'debug':
	env.MergeFlags({'CPPDEFINES':['DEBUG', 'TRACING_ON=1']})
if build_mode == 'release':
	env.MergeFlags({'CPPDEFINES':['TRACING_ON=1']})
if build_mode == 'fast':
	env.MergeFlags({'CPPDEFINES':['NDEBUG', 'TRACING_ON=0']}) 

if sys.platform.startswith("win"):
    env.MergeFlags({'CPPDEFINES':['USE_BOOST_PROG_OPT','WIN32','NOMINMAX']})
    
# setting flags

flags= { 'CCFLAGS' : [ '--param inline-unit-growth=5000' ] }
linux_debug_flags={ 'CCFLAGS' : ['-Wno-non-template-friend','-Wno-deprecated','-ggdb','-rdynamic','-O0','-fno-inline'] }
linux_release_flags={ 'CCFLAGS' : ['-Wno-non-template-friend','-Wno-deprecated','-O2'] }
linux_debug_release_flags={ 'CCFLAGS' : ['-Wno-non-template-friend','-Wno-deprecated','-O2','-ggdb'] }
linux_fast_flags={ 'CCFLAGS' : ['-Wno-non-template-friend','-Wno-deprecated','-O3'] }



windows_debug_flags={ 'CCFLAGS' : ['/Od','/MTd','/EHsc','/D_SCL_SECURE_NO_WARNINGS'] }
windows_release_flags={ 'CCFLAGS' : ['/O2','/MT','/EHsc','/D_SCL_SECURE_NO_WARNINGS'] }

if sys.platform.startswith("linux"):
	if build_mode == 'debug':
		env.MergeFlags(linux_debug_flags)
	if build_mode == 'release':
		env.MergeFlags(linux_release_flags)
	elif build_mode == 'fast':
		env.MergeFlags(linux_fast_flags)
	# c++0x support in gcc is useful already from 4.4, see
	# http://gcc.gnu.org/projects/cxx0x.html for details
	if compareVersions(env['CXXVERSION'], '4.4') >= 0:
		env.MergeFlags( {'CCFLAGS' : ['-std=c++0x'] })

elif sys.platform.startswith("win"):
	if build_mode == 'debug':
		env.MergeFlags(windows_debug_flags)
		env['PDB']='nerios.pdb'
	else:
		env.MergeFlags(windows_release_flags)
	

# libapth
# libpath = [ 'tools'+os.sep+systemc+os.sep+'msvc71'+os.sep+'SystemC'+os.sep+'Debug','tools'+os.sep+mysqlpp+os.sep+'vc2008'+os.sep+'Debug','tools'+os.sep+xerces_c+os.sep+'Build'+os.sep+'Win32'+os.sep+projectcxxstring+os.sep+'Debug','C:\\Program Files\\MySQL\\MySQL Server 5.1\\lib\\debug']
		

#defining the targets
#debug = env.Program(target='nerios', source=src_files,CPPPATH=include_files, LINKFLAGS="--static")
#release = env.Program(target='nerios', source=src_files, LINKFLAGS="--static")
#defining number of gem5 libraries
gem5_num_exts = 1

#for target in components:
#    obj_path='components'+os.sep+target+os.sep+'lib'+os.sep+postfix 
#    libpath.append(obj_path)
    

target='gem5'
env.SConscript('SConscript', duplicate=0, exports='target env build_mode gem5_num_exts')
env.SharedLibrary(target = 'armA15_systemC', source = [ Glob('gem5_ArmA15'+ os.sep + 'src' + os.sep +'*.cpp')])

#obj_path='components'+os.sep+'gem5'+os.sep+'lib_ext'+os.sep+postfix
#myScript=SConscript('components'+os.sep+'gem5'+os.sep+'SConscript', variant_dir=obj_path, duplicate=0, exports='build_mode env')
libpath = ['gem5'+os.sep+'build'+os.sep+'ARM']
libpath.append(py_lib_path)
libpath.append('gem5'+os.sep+'build'+os.sep+'gzstream')
libpath.append('gem5'+os.sep+'build'+os.sep+'libelf')

	
    #env.Depends('nerios',myScript)

# Handle circular dependencies in the libraries
#if sys.platform.startswith("linux"):
#    env['_LIBFLAGS'] = ' -L. -static-libgcc -Wl,--start-group ' + env['_LIBFLAGS'] 
#    env['_LIBFLAGS'] = env['_LIBFLAGS'] + ' -Wl,--whole-archive '
#    for i in range(gem5_num_exts):
#        env['_LIBFLAGS'] = env['_LIBFLAGS'] +' -lgem5_ext' + str(i)
#    env['_LIBFLAGS'] = env['_LIBFLAGS'] + ' -Wl,--no-whole-archive -Wl,--end-group'

#env.VariantDir('kernel'+os.sep+'nerios'+os.sep+'obj'+os.sep+postfix,'kernel'+os.sep+'nerios'+os.sep+'src', duplicate=0)
#nerios_prog=env.Program('kernel'+os.sep+'nerios'+os.sep+'obj'+os.sep+postfix+os.sep+'nerios.cpp', LIBS=kernel+components+tools_libs+third_part_libs+system_libs, LIBPATH = libpath , LINKFLAGS="-nodefaultlibs -Wl,-as-needed")#, LINKFLAGS="--static")   
#if sys.platform.startswith("linux"):
    # This is a work-around for the fact that SCons doesn't automatically add dependencies for the libraries mentioned in LINKCOM.
#	for i in range(gem5_num_exts):
#		 LibFile = env.FindFile('libgem5_ext'+ str(i)+'.a', libpath)
#		 env.Depends(nerios_prog, LibFile)	

#env.Alias('nerios',nerios_prog)

#if sys.platform.startswith("linux"):	
#    env.Alias('install',env.InstallAs(target='bin'+os.sep+postfix+os.sep+'nerios', source='kernel'+os.sep+'nerios'+os.sep+'obj'+os.sep+postfix+os.sep+'nerios'))
#if sys.platform.startswith("win"):	
#    env.Alias('install',env.InstallAs(target='bin'+os.sep+postfix+os.sep+'nerios.exe', source='kernel'+os.sep+'nerios'+os.sep+'obj'+os.sep+postfix+os.sep+'nerios.exe'))

