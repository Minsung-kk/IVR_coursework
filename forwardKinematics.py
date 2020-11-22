import numpy as np

# degree to radian
def DtoR(x):
    return (x/180.0)*np.pi

time = [x for x in range(1,11)]
# print the end-effector position for t where t = 1 to 10       
for t in time:
    # sinusoidal signals in radians. t is time
    j1 = DtoR(180*np.sin(np.pi/15*t))
    j2 = DtoR(90*np.sin(np.pi/15*t))
    j3 = DtoR(90*np.sin(np.pi/18*t))
    j4 = DtoR(90*np.sin(np.pi/20*t))

    # Denavit-Hartenberg Convention
    # using sinusoidal signals j
    # parameter table [theta, alpha, r, d], l is link
    l1 = [0+DtoR(90), DtoR(90), 0, 2.5]
    l2 = [j2+DtoR(90), DtoR(90), 0, 0]
    l3 = [j3, -DtoR(90), 3.5, 0]
    l4 = [j4, 0, 3, 0]

    # test for simple angles
    # degrees
    # t1 = 0
    # t2 = 0
    # t3 = 90
    # t4 = 90
    # l1 = [DtoR(t1)+DtoR(90), DtoR(90), 0, 2.5]
    # l2 = [DtoR(t2)+DtoR(90), DtoR(90), 0, 0]
    # l3 = [DtoR(t3), -DtoR(90), 3.5, 0]
    # l4 = [DtoR(t4), 0, 3, 0]

    Ls = np.array([l1,l2,l3,l4])

    Rs = [] 

    # Form the homogeneous transformation matrices for each t
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

    # transformation
    for i in Rs:
        temp = np.dot(temp, i)

    # print(np.dot(np.dot( np.dot(Rs[0], Rs[1]), Rs[2]) ,Rs[3]))

    # print x, y, z positions of the end-effector at t
    print("end-effector position for t = " + str(t))
    print("x: " + str(temp[0][3]))
    print("y: " + str(temp[1][3]))
    print("z: " + str(temp[2][3]))
