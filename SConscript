

def chunks(l, n):
    """ Yield successive n-sized chunks from l.
    """
    for i in xrange(0, len(l), n):
        yield l[i:i+n]

#imports 

Import('target env build_mode gem5_num_exts')

if (target == 'gem5'):
    import os
    src_files = []
    if (build_mode == 'debug'):
        source = Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*.do') + \
        Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*.do')  + \
        Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.do')  + \
        Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.do') + \
        Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.do') 
        source2=list(chunks(source,int(len(source)/gem5_num_exts+1)))
        for l in source2:
            src_files.append([ elem for elem in l if ((elem.name != "main.do") & (elem.name != "eventq.do") & (elem.name != "pollevent.do") & (elem.name != "sim_object.do") & (elem.name != "stats_wrap.do") & (elem.name != "stats.i_init.do")) ])
        for i in range(len(src_files)):
            env.SharedLibrary('gem5'+os.sep+'lib'+os.sep+'gem5_dbg'+str(i),src_files[i])
#            print "creating dep for " + 'gem5_ext'+str(i)
    if (build_mode == 'release'):
        source =  Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*.o') + \
            Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*.o') + \
            Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.o') + \
            Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.o') + \
            Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.o') 
        source2=list(chunks(source,int(len(source)/gem5_num_exts+1)))
        for l in source2:
            src_files.append([ elem for elem in l if ((elem.name != "main.o") & (elem.name != "eventq.o") & (elem.name != "pollevent.o") & (elem.name != "sim_object.o") & (elem.name != "stats_wrap.o") & (elem.name != "stats.i_init.o")) ])  # TODO: filter only needed py elements using sth like & ((re.match('.*.py.do',elem.name) == None) | (elem.name == "importer.py.do") | (elem.name == "__init__.py.do")) & ((re.match('.*.i_init.do',elem.name) == None) | (elem.name == "stats.i_init.do"))) ] 
        for elem1 in src_files:
            for elem2 in elem1:
                elem2.attributes.shared = True
        for i in range(len(src_files)):
            env.SharedLibrary('gem5'+os.sep+'lib'+os.sep+'gem5_rel'+str(i),src_files[i])
    if (build_mode == 'fast'):
        source =  Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*.fo') + \
            Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*.fo') + \
            Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.fo') + \
            Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.fo') + \
            Glob('gem5'+os.sep+'build'+os.sep+'ARM'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*'+os.sep+'*.fo') 
        source2=list(chunks(source,int(len(source)/gem5_num_exts+1)))
        for l in source2:
            src_files.append([ elem for elem in l if ((elem.name != "main.fo") & (elem.name != "eventq.fo") & (elem.name != "pollevent.fo") & (elem.name != "sim_object.fo") & (elem.name != "stats_wrap.fo") & (elem.name != "stats.i_init.fo")) ])  # TODO: filter only needed py elements using sth like & ((re.match('.*.py.do',elem.name) == None) | (elem.name == "importer.py.do") | (elem.name == "__init__.py.do")) & ((re.match('.*.i_init.do',elem.name) == None) | (elem.name == "stats.i_init.do"))) ] 
        for i in range(len(src_files)):
            env.SharedLibrary('gem5'+os.sep+'lib'+os.sep+'gem5_fst'+str(i),src_files[i])
#            print "creating dep for " + 'gem5_ext'+str(i)
    
