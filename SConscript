

def chunks(l, n):
    """ Yield successive n-sized chunks from l.
    """
    for i in xrange(0, len(l), n):
        yield l[i:i+n]

#imports 

Import('target env build_mode gem5_num_exts')

if (target == 'gem5'):
    import fnmatch
    import os
    src_files = []
    if (build_mode == 'debug'):
        source = []
        for root, dirnames, filenames in os.walk('gem5'+os.sep+'sysc_build'+os.sep+'ARM'):
            for filename in fnmatch.filter(filenames, '*.do'):
                source.append(File(os.path.join(root, filename)))
        source2=list(chunks(source,int(len(source)/gem5_num_exts+1)))
        for l in source2:
            src_files.append([ elem for elem in l if ((elem.name != "main.do") & (elem.name != "sim_object.do") & (elem.name != "stats_wrap.do") & (elem.name != "stats.i_init.do")) ])
        for elem1 in src_files:
            for elem2 in elem1:
                elem2.attributes.shared = True
        for i in range(len(src_files)):
            env.SharedLibrary('gem5'+os.sep+'lib'+os.sep+'gem5_dbg'+str(i),src_files[i])
    if (build_mode == 'release'):
        source = []
        for root, dirnames, filenames in os.walk('gem5'+os.sep+'sysc_build'+os.sep+'ARM'):
            for filename in fnmatch.filter(filenames, '*.o'):
                source.append(File(os.path.join(root, filename)))
        source2=list(chunks(source,int(len(source)/gem5_num_exts+1)))
        for l in source2:
            src_files.append([ elem for elem in l if ((elem.name != "main.o") & (elem.name != "sim_object.o") & (elem.name != "stats_wrap.o") & (elem.name != "stats.i_init.o")) ]) 
        for elem1 in src_files:
            for elem2 in elem1:
                elem2.attributes.shared = True
        for i in range(len(src_files)):
            env.SharedLibrary('gem5'+os.sep+'lib'+os.sep+'gem5_rel'+str(i),src_files[i])
    if (build_mode == 'fast'):
        source = []
        for root, dirnames, filenames in os.walk('gem5'+os.sep+'sysc_build'+os.sep+'ARM'):
            for filename in fnmatch.filter(filenames, '*.fo'):
                source.append(File(os.path.join(root, filename)))
        source2=list(chunks(source,int(len(source)/gem5_num_exts+1)))
        for l in source2:
            src_files.append([ elem for elem in l if ((elem.name != "main.fo") & (elem.name != "sim_object.fo") & (elem.name != "stats_wrap.fo") & (elem.name != "stats.i_init.fo")) ]) 
        for elem1 in src_files:
            for elem2 in elem1:
                elem2.attributes.shared = True
        for i in range(len(src_files)):
            env.SharedLibrary('gem5'+os.sep+'lib'+os.sep+'gem5_fst'+str(i),src_files[i])
#            print "creating dep for " + 'gem5_ext'+str(i)
    
