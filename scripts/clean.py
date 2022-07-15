#!/usr/bin/env python3


file1 = open("path.txt", "r")
Lines = file1.readlines()
file2 = open("path.txt","w")
newLines = []
for i in range(len(Lines)):
    if i%3==0:
        newLines.append(Lines[i])

file2.writelines(newLines)
