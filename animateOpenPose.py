import pandas as pd
import numpy as np
import math
def get_openpose_df(openpose_file):
    openpose_df = pd.read_csv(openpose_file);
    
    #keep wrist and neck data so far
    columns_sel =['secs', 'nsecs', 'lwrist.x', 'lwrist.y', 'rwrist.x', 'rwrist.y', 'neck.x', 'neck.y',
            'lelbow.x', 'lelbow.y','relbow.x', 'relbow.y', 'lshoulder.x', 'lshoulder.y',
            'rshoulder.x', 'rshoulder.y']
    openpose_df = openpose_df[columns_sel]; 
    openpose_df=openpose_df.rename(index=str, columns={'secs':'Seconds','nsecs':'NanoSeconds'})

    time = []
    starttime=np.round(openpose_df['Seconds'][0]+openpose_df['NanoSeconds'][0]/(10.0**9),13) ##Seconds from 0
    r = len(openpose_df)
    for i in range(r):
        time.append(np.round(openpose_df['Seconds'][i]+openpose_df['NanoSeconds'][i]/(10.0**9),13))
    time = time-starttime
    openpose_df.insert(2, 'time', time, True)
    
    #try to fill as many NAs as possbile
    for i in range(3,len(openpose_df.columns)):
        hitna = False
        num = 0
        prev = 0
        name = openpose_df.columns[i]
        for j in range(len(openpose_df)):
            if (math.isnan(openpose_df.iloc[j][i]) and (not hitna)):
                hitna = True
                num = 1
                prev = j-1
            elif (math.isnan(openpose_df.iloc[j][i]) and hitna):
                num += 1
            elif (hitna):
                hitna = False
                dist = (openpose_df.iloc[j][i]-openpose_df.iloc[prev][i])
                timeLen = np.round(openpose_df['time'][j]-openpose_df['time'][prev],13)
                for k in (1, num+1):
                    openpose_df[name][prev+k] = int(openpose_df.iloc[prev][i]+dist*((openpose_df['time'][prev+k]-openpose_df['time'][prev])/timeLen))
                
    openpose_df = openpose_df.dropna()
 
    return openpose_df

def get_points(openpose_df):
    for i in range(3, len(openpose_df.columns),2):
        name1 = openpose_df.columns[i]
        name2 = openpose_df.columns[i+1]
        openpose_df[name1] = 640-openpose_df[name1]
        openpose_df[name2] = 480-openpose_df[name2]
    return openpose_df


def interpolate(openpose_df):
    columns_sel = ['time', 'lwrist.x', 'lwrist.y', 'rwrist.x', 'rwrist.y', 'neck.x', 'neck.y','lelbow.x', 'lelbow.y',
            'relbow.x', 'relbow.y', 'lshoulder.x', 'lshoulder.y', 'rshoulder.x', 'rshoulder.y']
    points_df = openpose_df[columns_sel]
    
    lx = []
    ly = []
    rx = []
    ry = []
    nx = []
    ny = []
    lex = []
    ley = []
    rex = []
    rey = []
    lsx = []
    lsy = []
    rsx = []
    rsy =[]
    time = []
    gap = []

    interval = 1.0/30

    #grab last time point
    lasttime = int(points_df['time'].iloc[len(points_df)-1])
    
    #this is how many points we need for each frame
    for i in range(lasttime):
        for j in range(30):
            time.append(i+j*interval)

    #for every point that we have, either a: fill it up, or b, guess and interpolate
    #size of time
    timelength = lasttime*30
    pointslength = len(points_df)
    num = 0
    j = 1 #keep track of j, which is the index of points_df

    a = 0
    while time[a] < points_df['time'].iloc[0]:
        lx.append(points_df['lwrist.x'].iloc[0])
        ly.append(points_df['lwrist.y'].iloc[0])
        rx.append(points_df['rwrist.x'].iloc[0])
        ry.append(points_df['rwrist.y'].iloc[0])
        nx.append(points_df['neck.x'].iloc[0])
        ny.append(points_df['neck.y'].iloc[0])
        lex.append(points_df['lelbow.x'].iloc[0])
        ley.append(points_df['lelbow.y'].iloc[0])
        rex.append(points_df['relbow.x'].iloc[0])
        rey.append(points_df['relbow.y'].iloc[0])
        lsx.append(points_df['lshoulder.x'].iloc[0])
        lsy.append(points_df['lshoulder.y'].iloc[0])
        rsx.append(points_df['rshoulder.x'].iloc[0])
        rsy.append(points_df['rshoulder.y'].iloc[0])
        gap.append(False)
        a+=1

    
    #print(time)
    for i in range(a,timelength):
        if(j >= pointslength):
            break

        first = points_df['time'].iloc[j-1]
        second = points_df['time'].iloc[j]
        
        #at first time at which the second time is equal or less
        if(second <= time[i]):
            #getting all the points and distances
            lxfirst = points_df['lwrist.x'].iloc[j-1]
            lyfirst = points_df['lwrist.y'].iloc[j-1]
            lxsecond = points_df['lwrist.x'].iloc[j]
            lysecond = points_df['lwrist.y'].iloc[j]
            lxdist = int((lxfirst-lxsecond)/(num+1))
            lydist = int((lyfirst-lysecond)/(num+1))
            
            rxfirst = points_df['rwrist.x'].iloc[j-1]
            ryfirst = points_df['rwrist.y'].iloc[j-1]
            rxsecond = points_df['rwrist.x'].iloc[j]
            rysecond = points_df['rwrist.y'].iloc[j]
            rxdist = int((rxfirst - rxsecond)/(num+1))
            rydist = int((ryfirst - rysecond)/(num+1))
        
            nxfirst = points_df['neck.x'].iloc[j-1]
            nyfirst = points_df['neck.y'].iloc[j-1]
            nxsecond = points_df['neck.x'].iloc[j]
            nysecond = points_df['neck.y'].iloc[j]
            nxdist = int((nxfirst - nxsecond)/(num+1))
            nydist = int((nyfirst - nysecond)/(num+1))

            lexfirst = points_df['lelbow.x'].iloc[j-1]
            leyfirst = points_df['lelbow.y'].iloc[j-1]
            lexsecond = points_df['lelbow.x'].iloc[j]
            leysecond = points_df['lelbow.y'].iloc[j]
            lexdist = int((lexfirst-lexsecond)/(num+1))
            leydist = int((leyfirst-leysecond)/(num+1))
            
            rexfirst = points_df['relbow.x'].iloc[j-1]
            reyfirst = points_df['relbow.y'].iloc[j-1]
            rexsecond = points_df['relbow.x'].iloc[j]
            reysecond = points_df['relbow.y'].iloc[j]
            rexdist = int((rexfirst - rexsecond)/(num+1))
            reydist = int((reyfirst - reysecond)/(num+1))

            lsxfirst = points_df['lshoulder.x'].iloc[j-1]
            lsyfirst = points_df['lshoulder.y'].iloc[j-1]
            lsxsecond = points_df['lshoulder.x'].iloc[j]
            lsysecond = points_df['lshoulder.y'].iloc[j]
            lsxdist = int((lsxfirst-lsxsecond)/(num+1))
            lsydist = int((lsyfirst-lsysecond)/(num+1))
            
            rsxfirst = points_df['rshoulder.x'].iloc[j-1]
            rsyfirst = points_df['rshoulder.y'].iloc[j-1]
            rsxsecond = points_df['rshoulder.x'].iloc[j]
            rsysecond = points_df['rshoulder.y'].iloc[j]
            rsxdist = int((rsxfirst - rsxsecond)/(num+1))
            rsydist = int((rsyfirst - rsysecond)/(num+1))

            
            for k in (range(1, num+1)):
                lx.append(int(lxfirst+lxdist*k))
                ly.append(int(lyfirst+lydist*k))
                rx.append(int(rxfirst+rxdist*k))
                ry.append(int(ryfirst+rydist*k))
                nx.append(int(nxfirst+nxdist*k))
                ny.append(int(nyfirst+nydist*k))
                lex.append(int(lexfirst+lexdist*k))
                ley.append(int(leyfirst+leydist*k))
                rex.append(int(rexfirst+rexdist*k))
                rey.append(int(reyfirst+reydist*k))
                lsx.append(int(lsxfirst+lsxdist*k))
                lsy.append(int(lsyfirst+lsydist*k))
                rsx.append(int(rsxfirst+rsxdist*k))
                rsy.append(int(rsyfirst+rsydist*k))
                if(num > 45):
                    gap.append(True)
                else:
                    gap.append(False)
 
            lx.append(lxsecond)
            ly.append(lysecond)
            rx.append(rxsecond)
            ry.append(rysecond)
            nx.append(nxsecond)
            ny.append(nysecond)
            lex.append(lexsecond)
            ley.append(leysecond)
            rex.append(rexsecond)
            rey.append(reysecond)
            lsx.append(lsxsecond)
            lsy.append(lsysecond)
            rsx.append(rsxsecond)
            rsy.append(rsysecond)
            gap.append(False)

            j+=1
            num = 0
        else:
            num +=1


    #print(len(lx))
    while(len(lx) < timelength):
        lx.append(points_df['lwrist.x'].iloc[pointslength-1])
        ly.append(points_df['lwrist.y'].iloc[pointslength-1])
        rx.append(points_df['rwrist.x'].iloc[pointslength-1])
        ry.append(points_df['rwrist.y'].iloc[pointslength-1])
        nx.append(points_df['neck.x'].iloc[pointslength-1])
        ny.append(points_df['neck.y'].iloc[pointslength-1])
        lex.append(points_df['lelbow.x'].iloc[pointslength-1])
        ley.append(points_df['lelbow.y'].iloc[pointslength-1])
        rex.append(points_df['relbow.x'].iloc[pointslength-1])
        rey.append(points_df['relbow.y'].iloc[pointslength-1])
        lsx.append(points_df['lshoulder.x'].iloc[pointslength-1])
        lsy.append(points_df['lshoulder.y'].iloc[pointslength-1])
        rsx.append(points_df['rshoulder.x'].iloc[pointslength-1])
        rsy.append(points_df['rshoulder.y'].iloc[pointslength-1])
        gap.append(False)
 
    return pd.DataFrame({'time': time, 'lwrist.x':lx, 'lwrist.y':ly, 'rwrist.x': rx, 'rwrist.y': ry, 'neck.x': nx, 'neck.y':ny,
        'lelbow.x':lex, 'lelbow.y':ley, 'relbow.x':rex, 'relbow.y':rey, 'lshoulder.x': lsx, 'lshoulder.y':lsy, 'rshoulder.x':rsx, 'rshoulder.y':rsy, 'gap':gap}) 


        




