import os 
import re
import time

def comment_remover(text): 
    def replacer(match): 
        s = match.group(0) 
        if s.startswith('/'): 
            return "" 
        else: 
            return s 

    pattern = re.compile( 
        r'//.*?$|/\*.*?\*/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"', 
        re.DOTALL | re.MULTILINE 
    ) 
    return re.sub(pattern, replacer, text) 

def file_check(name):
	if name.endswith('.c') or name.endswith('.java') or name.endswith('.cpp'):
		total_line = 0
		write_line = 0
		writefilename = name + ".bak"
		readfile = open(name, "r")
		writefile = open(writefilename, "w")
		while readfile:
			line = readfile.readline()
			if not line: break
			total_line += 1
			writefile.writelines(comment_remover(line))

		readfile.close()
		writefile.close()
		print name, " file is \t\t", total_line, " lines"
	return

def file_list(dir): 

	for name in os.listdir(dir):
		checkname = name
		checkname = checkname.replace('.', '')
		checkname = checkname.replace('/', '')
		if checkname == 'git' or checkname == 'repo':
			continue
		
		fullname = os.path.join(dir, name)
		if os.path.isdir(fullname):
			file_list(fullname)
		else:
			file_check(fullname)
		
	return

print "===================================================================="
print "===               Program of Delete comments by Owl              ==="
print "===================================================================="

print "=== start time : ", time.ctime() 
file_list('.') 

