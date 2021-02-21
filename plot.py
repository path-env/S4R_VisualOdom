import rosbag
import matplotlib.pyplot as plt

bag = rosbag.Bag('/home/bheeshma/2021-02-19-17-25-02.bag') #508 - 620

x_traject = []
y_traject = []
for topic, msg, t in bag.read_messages(topics=['/rover/DistanceDriven']):
    
        
    if  topic == '/rover/DistanceDriven':
        dat = msg.data
        temp = dat.split(',')
        x =  temp[0].split(':')[1]
        y =  temp[1].split(':')[1]
        x_traject.append(round(float(x),2))
        y_traject.append(round(float(y),2))

plt.scatter(y_traject,x_traject)
plt.xlabel("Y")
plt.ylabel("x")
plt.show()

