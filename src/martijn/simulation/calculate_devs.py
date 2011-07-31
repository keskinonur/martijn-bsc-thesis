import sys, math

if len(sys.argv) == 1:
  print("Usage: "+sys.argv[0]+" <results> <means>")
  exit()

def find_match(means, stat):
  for mean in means:
    if mean[0] == stat:
      return float(mean[1])
  return None


fp = open(sys.argv[1], 'r')
fp2 = open(sys.argv[2], 'r')

stats = [x.split() for x in fp.readlines()]
means = [x.split() for x in fp2.readlines()]

fp.close()
fp2.close()

curr = "x:"
count = 1
total = 0
for i in range(len(stats)):
  if stats[i][0] != curr:
    # print last stat
    dev = total / count
    print str(curr) + " " + str(dev) + " [" + str(count) + "]"
    # reset
    curr = stats[i][0]
    count = 1
    mean = find_match(means, curr)
    total = math.pow(mean - float(stats[i][1]), 2)
  else:
    count += 1
    total += math.pow(mean - float(stats[i][1]), 2)

# print last stat
mean = 1.0*total/count
print str(curr) + " " + str(mean) + " [" + str(count) + "]"
