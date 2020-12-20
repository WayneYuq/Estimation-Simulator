import numpy as np

if __name__ == '__main__':
    with open('../config/log/Graph2.txt', 'r') as f:
	lines = f.readlines()
	lines.pop(0)
	newline = []
	for line in lines:
	    line = line.rstrip().split(',')[1]
	    num = float(line)
	    newline.append(num)
	nnl = np.array(newline)
	print(nnl)
	print(np.std(nnl))