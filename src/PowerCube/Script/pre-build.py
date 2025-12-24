import io
import os
from os import path
import shutil


def update_version(name):
	# backup current file
	backup = name + ".bak"
	if path.exists(backup):
		os.remove(backup)
		
	shutil.copyfile(name, backup)
	
	# read all
	with open(name, "r") as f:
		data = f.readlines()

	# update build-number
	file = open(name, "w")

	for line in data:
		words = line.split()
		
		if (len(words) > 2 and words[0] == '#define' and words[1] == 'FW_VERSION_MAJOR'):
			major = int(words[2])
			file.write('#define FW_VERSION_MAJOR ' + str(major) + '\n')
			print ('FW_VERSION_MAJOR=' + str(major))
		elif (len(words) > 2 and words[0] == '#define' and words[1] == 'FW_VERSION_MINOR'):
			minor = int(words[2])
			file.write('#define FW_VERSION_MINOR ' + str(minor) + '\n')
			print ('FW_VERSION_MINOR=' + str(minor))
		elif (len(words) > 2 and words[0] == '#define' and words[1] == 'FW_VERSION_PATCH'):
			patch = int(words[2])
			file.write('#define FW_VERSION_PATCH ' + str(patch) + '\n')
			print ('FW_VERSION_PATCH=' + str(patch))
		elif (len(words) > 2 and words[0] == '#define' and words[1] == 'FW_VERSION_SUFFIX'):
			suffix = words[2]
			if (suffix.isnumeric()):
				suffixNew = str(int(suffix) + 1)
			else:
				suffixNew = suffix
			file.write('#define FW_VERSION_SUFFIX ' + suffixNew + '\n')
			print ('FW_VERSION_SUFFIX=' + suffixNew)
		else:
			for word in words:
				file.write(word)
				file.write(' ')
			file.write('\n')

	file.close()
	
	# make version string
	v1 = "\"" + str(major) + "." + str(minor) + "." + str(patch) + "-" + str(suffix) + "\""
	v2 = "\"" + str(major) + "." + str(minor) + "." + str(patch) + "-" + str(suffixNew) + "\""
	print ("version1: " + v1);
	print ("version2: " + v2); 
	
	return [v1, v2]



#
#
#

vfile = "../Core/Inc/version.h"

version = update_version(vfile)
