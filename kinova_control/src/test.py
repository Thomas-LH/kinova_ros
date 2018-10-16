#! /usr/bin/env python
from copy import deepcopy

def tri():
    d=[1]
    t=[]
    for index in range(9):
        t=deepcopy(d)
        yield t
        d.append(1)
        if len(d) > 2:
            for j in range(len(d)-2):
                d[j+1] = t[j] + t[j+1]
    

if __name__ == '__main__':
    for n in tri():
        print(n)