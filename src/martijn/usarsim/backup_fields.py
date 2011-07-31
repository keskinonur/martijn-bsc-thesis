import os, shutil, sys, time

if len(sys.argv) > 1 and sys.argv[1] == "r":
  
  # restore last

  l = [ (os.path.getmtime('../backup/'+x), x) for x in os.listdir('../backup/') ]
  l.sort()
  from1 = '../backup/' + l[-1][1]
  from2 = '../backup/' + l[-2][1]

  if 'l2r' in from1:
    from1, from2 = from2, from1

  to1 = '../stage_r2l.txt'
  to2 = '../stage_l2r.txt'

  shutil.copyfile(from1, to1)
  shutil.copyfile(from2, to2)

else:
  
  # backup

  from1 = '../stage_r2l.txt'
  from2 = '../stage_l2r.txt'

  to1 = '../backup/stage_r2l_' + str(time.time()) + '.txt'
  to2 = '../backup/stage_l2r_' + str(time.time()) + '.txt'

  shutil.copyfile(from1, to1)
  shutil.copyfile(from2, to2)

