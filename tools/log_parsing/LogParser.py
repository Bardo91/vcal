#!/usr/bin/env python

import numpy

class LogParser:
	def __init__(self, file_path):
		self.file_path = file_path

	def listTags(self):
		logFile = open(self.file_path, 'r')
		line = logFile.readline()
		tags = []
		while line:
			idx1 = line.find("[")
			idx2 = line.find("]")
			tag = line[idx1+1:idx2]
			if tag not in tags:			
				tags.append(tag)
			line = logFile.readline()
		
		return tags

	def vectorFloat(self, tag_name):
		logFile = open(self.file_path, 'r')
		line = logFile.readline()
		vectorData = []
		while line:
			if tag_name in line:
				idx = line.find(tag_name) + len(tag_name)+1
				number = float(line[idx:])
				vectorData.append(number)
			line = logFile.readline()
		
		return vectorData

	def vector3Float(self, tag_name):
		logFile = open(self.file_path, 'r')
		line = logFile.readline()
		vectorDataX = []
		vectorDataY = []
		vectorDataZ = []
		while line:
			if tag_name in line:
				idx = line.find(tag_name) + len(tag_name)+1
				numbers = line[idx:]
				if "nan" not in numbers:
					elems = numbers.split(',')				
					vectorDataX.append(float(elems[0]))
					vectorDataY.append(float(elems[1]))
					vectorDataZ.append(float(elems[2]))
			line = logFile.readline()
		
		return vectorDataX, vectorDataY, vectorDataZ


