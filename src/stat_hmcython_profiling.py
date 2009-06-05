import pstats
p1 = pstats.Stats('h_c_statfile1')
#p2 = pstats.Stats('h_c_statfile2')
p1.sort_stats('time').print_stats(.25)
#p2.sort_stats('time').print_stats(.25)
