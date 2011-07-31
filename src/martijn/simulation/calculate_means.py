fp = open('backup_stats.txt', 'r')

stats = [x.split() for x in fp.readlines()]

fp.close()

curr = "x:"
count = 1
total = 0
for i in range(len(stats)):
  if stats[i][0] != curr:
    # print last stat
    mean = 1.0*total/count
    print str(curr) + " " + str(mean) + " [" + str(count) + "]"
    # reset
    curr = stats[i][0]
    count = 1
    total = float(stats[i][1])
  else:
    count += 1
    total += float(stats[i][1])

# print last stat
mean = 1.0*total/count
print str(curr) + " " + str(mean) + " [" + str(count) + "]"
