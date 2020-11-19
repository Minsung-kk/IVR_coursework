import numpy as np

# degrees
t1 = 0
t2 = 0
t3 = 90
t4 = 90

# degree to radian
def DtoR(x):
    return (x/180.0)*np.pi

# sinusoidal signals
t = 1;
j1 = DtoR(180*np.sin(np.pi/15*t))
j2 = DtoR(90*np.sin(np.pi/15*t))
j3 = DtoR(90*np.sin(np.pi/18*t))
j4 = DtoR(90*np.sin(np.pi/20*t))

# theta, alpha, r, d 
# l1 = [DtoR(t1)+DtoR(90), DtoR(90), 0, 2.5]
# l2 = [DtoR(t2)+DtoR(90), DtoR(90), 0, 0]
# l3 = [DtoR(t3), -DtoR(90), 3.5, 0]
# l4 = [DtoR(t4), 0, 3, 0]

l1 = [0+DtoR(90), DtoR(90), 0, 2.5]
l2 = [j2+DtoR(90), DtoR(90), 0, 0]
l3 = [j3, -DtoR(90), 3.5, 0]
l4 = [j4, 0, 3, 0]
print(j2)
print(j3)
print(j4)

Ls = np.array([l1,l2,l3,l4])

# print(Ls[1])

Rs = [] 

for i in Ls:
    Rs.append(np.dot(
        [
         [np.cos(i[0]), -np.sin(i[0]),0,0], 
         [np.sin(i[0]), np.cos(i[0]),0,0],
         [0,0,1,i[3]],
         [0,0,0,1],
        ], 
        [
         [1,0,0,i[2]],
         [0,np.cos(i[1]),-np.sin(i[1]),0],
         [0,np.sin(i[1]),np.cos(i[1]),0],
         [0,0,0,1]
        ]))

temp = np.identity(4)
# print(Rs)
for i in Rs:
    temp = np.dot(temp, i)

# print(np.dot(np.dot( np.dot(Rs[0], Rs[1]), Rs[2]) ,Rs[3]))
# print()
print(temp)
