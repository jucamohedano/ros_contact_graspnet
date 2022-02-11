from mlsocket import MLSocket
import numpy as np
from time import sleep

HOST = '158.176.76.55'
PORT = 65432

# Make an ndarray
data = np.load('tiago_pcl.npy')

# Send data
with MLSocket() as s:
    s.connect((HOST, PORT)) # Connect to the port and host
    print('sent data!')
    r = s.send(data) # After sending the data, it will wait until it receives the reponse from the server
    res = s.recv(1024) # waits for message
    print(res)