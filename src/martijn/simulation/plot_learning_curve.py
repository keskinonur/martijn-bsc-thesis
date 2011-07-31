import matplotlib.pyplot as plot

fp = open('backup_stats_clean.txt', 'r')

stats = [x.split() for x in fp.readlines()]

fp.close()

curr = 0
keys = list()
vals = list()
for i in range(len(stats)):
  if int(stats[i][0][:-1]) == curr:
    print "double record: " + str(stats[i][0][:-1])
    exit()
  keys.append(int(stats[i][0][:-1]))
  vals.append(float(stats[i][1]))


plot.plot(keys, vals)
plot.title('Learning curve')
plot.xlabel('Transition number')
plot.ylabel('Duration')
plot.show()

