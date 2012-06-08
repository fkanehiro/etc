from subprocess import Popen, PIPE

p = Popen(["ls","-1"], stdout=PIPE)

while 1:
    l = p.stdout.readline()
    if l == "Makefile\n":
        print "found"
    if not l:
        break
    print '[',l,']',
p.wait()

