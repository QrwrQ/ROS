import csv
import matplotlib.pyplot as plt
import numpy as np
li_x=list()
li_y=list()
def cal_cov():
    with open('final_po.csv','rb') as f:
        csv_read=csv.reader(f)
        for line in csv_read:
            if line!=['x','y']:
                li_x.append(float(line[0]))
                li_y.append(float(line[1]))
            print line
    li_x=li_x[1:10]
    li_y=li_y[1:10]
    ar=np.vstack((li_x,li_y))
    print np.cov(ar)
def plot_trajectory():
    with open('open_po.csv','rb') as f:
        csv_read=csv.reader(f)
        for line in csv_read:
            if line!=['x','y']:
                li_x.append(float(line[0]))
                li_y.append(float(line[1]))
    li1=li_x[0:82]
    li2=li_x[83:164]
    li3=li_x[165:248]
    li4=li_x[249:332]
    li5=li_x[333:415]
    li6=li_x[416:499]
    li7=li_x[500:582]
    li8=li_x[583:664]
    li9=li_x[665:746]
    li10=li_x[747:829]

    liy1=li_y[0:82]
    liy2=li_y[83:164]
    liy3=li_y[165:248]
    liy4=li_y[249:332]
    liy5=li_y[333:415]
    liy6=li_y[416:499]
    liy7=li_y[500:582]
    liy8=li_y[583:664]
    liy9=li_y[665:746]
    liy10=li_y[747:829]

    plt.plot(li1,liy1)
    plt.plot(li2,liy2)
    plt.plot(li3,liy3)
    plt.plot(li4,liy4)
    plt.plot(li5,liy5)
    plt.plot(li6,liy6)
    plt.plot(li7,liy7)
    plt.plot(li8,liy8)
    plt.plot(li9,liy9)
    plt.plot(li10,liy10)
    plt.show()

    '''
    for i in range(0,len(li_x)):
        if (li_x[i]!=float(0))&(li_x[i+1]==float(0)):
            print i+1
    '''
if __name__ == '__main__':
    plot_trajectory()

