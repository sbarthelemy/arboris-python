from human36 import human36

(w,b,j) = human36()

for d in b['LPT'].descendants():
    print d.name

for d in b['Head'].ancestors():
    print d.name


