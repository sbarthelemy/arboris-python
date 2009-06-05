import pstats
p = pstats.Stats('h_statfile')
p.sort_stats('time').print_stats(.25)

