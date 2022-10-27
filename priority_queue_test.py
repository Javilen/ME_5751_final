# Test program for priority queue and dictionary testing
import queue
import itertools
#import numpy as np


# Works as intended (Breaks if you call somthing that isnt there)

# priority_Q=queue.PriorityQueue()

# priority_Q.put(69)
# priority_Q.put(24)
# priority_Q.put(2)
# priority_Q.put(90)
# priority_Q.put(42)
# priority_Q.put(22)
# priority_Q.put(6559)

# print(len(priority_Q))
# print(priority_Q.get())

# print(priority_Q.get())
# print(priority_Q.get())
# print(priority_Q.get())

# print(priority_Q.get())
# print(priority_Q.get())
# print(priority_Q.get())
class PriorityEntry(object):

    def __init__(self, priority, data):
        self.data = data
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority


#print(PriorityEntry(1, {'alpha': 1}) < PriorityEntry(0, {'beta': 2}))

count=itertools.count()


d={}

# d[3500]=[3,2,0]  # cost and the address of the cost
# d[2700]=[1,4,0]
# d[3600]=[5,6,0]
# d[24000]=[7,8,0]

pq=queue.PriorityQueue()

# we make a tuple consisting on costmap value, tiebreaker value as well as the address in reference to our vehicle map

pq.put([3500, next(count), [1,2]])
pq.put([3500, next(count), [1,2]])
pq.put([3400, next(count), [420,99]])
pq.put([3900, next(count), [10,20]])
pq.put([3500, next(count), [1,2]])

#print(pq.empty())
#print(pq.get())
path=[]
while pq.empty() == False: # prevents our hold condition for overdrawing from the queue
    value=pq.get()
    path.append(value[2])  # adds to our indicated path to the path list 
print(path)






#pq.put(PriorityEntry(1,{"x" : 25}))
# print(d)
# value=d.pop(5,None)
# print(value)






