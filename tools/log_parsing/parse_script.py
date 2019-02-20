#!/usr/bin/env python

from LogParser import *
import sys
import matplotlib.pyplot as plt

log_file = str(raw_input("Introduce log file: "))

parser = LogParser(log_file)

print(parser.listTags())

while True:
	tag = str(raw_input("Introduce tag or list of tags separated by commas: "))
	
	if ',' in tag:
		for x in tag.split(','):
			X, Y, Z = parser.vector3Float(x)
			if len(X) > 1:	
				plt.plot(X)
				plt.plot(Y)
				plt.plot(Z)
				plt.ylabel("time")
			else:
				print("Tag not found")
		plt.legend(tag.split(','))		
		plt.show()
	else:
		X, Y, Z = parser.vector3Float(tag)
		if len(X) > 1:	
			plt.plot(X)
			plt.plot(Y)
			plt.plot(Z)
			plt.ylabel("time")
			plt.show()
		else:
			print("Tag not found")

