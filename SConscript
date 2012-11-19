

#imports 

import os
Import('build_mode env')

source = []
target=''
if (build_mode == 'debug'):
    source2 =  Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*.do') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*.do') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.do') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.do') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.do')
    source = [ elem for elem in source2 if ((elem.name != "main.do") & (elem.name != "eventq.do") & (elem.name != "pollevent.do") & (elem.name != "sim_object.do") & (elem.name != "stats_wrap.do") & (elem.name != "stats.i_init.do")) ] # TODO: filter only needed py elements using sth like & ((re.match('.*.py.do',elem.name) == None) | (elem.name == "importer.py.do") | (elem.name == "__init__.py.do")) & ((re.match('.*.i_init.do',elem.name) == None) | (elem.name == "stats.i_init.do"))) ]
    target='gem5_debug'
if (build_mode == 'release'):
    source2 =  Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*.o') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*.o') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.o') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.o') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.o') 
    source = [ elem for elem in source2 if ((elem.name != "main.o") & (elem.name != "eventq.o") & (elem.name != "pollevent.o") & (elem.name != "sim_object.o") & (elem.name != "stats_wrap.o") & (elem.name != "stats.i_init.o")) ] # TODO: filter only needed py elements using sth like & ((re.match('.*.py.do',elem.name) == None) | (elem.name == "importer.py.do") | (elem.name == "__init__.py.do")) & ((re.match('.*.i_init.do',elem.name) == None) | (elem.name == "stats.i_init.do"))) ] 
    target='gem5_release'
if (build_mode == 'fast'):
    source2 =  Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*.fo') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*.fo') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.fo') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.fo') + \
            Glob('components'+os.sep+'gem5'+os.sep+'gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.fo') 
    source = [ elem for elem in source2 if ((elem.name != "main.fo") & (elem.name != "eventq.fo") & (elem.name != "pollevent.fo") & (elem.name != "sim_object.fo") & (elem.name != "stats_wrap.fo") & (elem.name != "stats.i_init.fo")) ] # TODO: filter only needed py elements using sth like & ((re.match('.*.py.do',elem.name) == None) | (elem.name == "importer.py.do") | (elem.name == "__init__.py.do")) & ((re.match('.*.i_init.do',elem.name) == None) | (elem.name == "stats.i_init.do"))) ] 
    target='gem5_fast'

env.StaticLibrary(target,source)
