import sys

if len(sys.argv) < 2:
	print "Usage: python euroc_to_tum.py <trajectory_filename>"
	exit(0)
filename = sys.argv[1]

file = open(filename)
data = file.read()
lines = data.replace(","," ").replace("\t"," ").split("\n") 
list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
# convert nanoseconds to seconds
list = map(lambda x: ["%.6f"%(x[0]/1e9)] + x[1], list)

with open(filename + '2', 'w') as file:
	file.writelines(' '.join(str(j) for j in i) + '\n' for i in list)