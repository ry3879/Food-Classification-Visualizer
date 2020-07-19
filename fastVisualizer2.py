import pandas as pd
import numpy as np
import matplotlib.backends.backend_tkagg as tkagg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import cv2
from PIL import Image
from PIL import ImageTk
from fastSegment2 import *
import Tkinter as tkinter
from ttk import *
import sys
from matplotlib.figure import Figure
from matplotlib.collections import LineCollection
from animateOpenPose import *


class App():
    #relevant variables pertaining to data from bags
    #relevant variables pertaining to data from video
    
    def __init__(self, master,g_df,v_file,p_df):
        gyro_df = g_df
        video_file = v_file
        points_df = p_df
        p_length = len(points_df) 
        #calculate what the size of the graphs should be
        self.width = int(master.winfo_screenwidth()//96//4)
        self.height = master.winfo_screenheight()//96//2-.25
        self.videoheight = int(self.width*96.0/640*480)
        self.videowidth = self.width*96 
        self.fig = Figure(figsize=(self.width, self.height))
        self.ax = self.fig.add_subplot(111)
        self.fig2 = Figure(figsize=(self.width, self.height))
        self.ax2 = self.fig2.add_subplot(111)

        plot_gyro_df(self, gyro_df)
        #initialize vertical lines
        self.ln1 = self.ax.axvline(0,0,.1)
        self.ln2 = self.ax2.axvline(0,0,.1)
        #first graph
        self.canvas1 = FigureCanvasTkAgg(self.fig,master=master)
        #self.canvas1.create_line(10,10,50,50)
        self.canvas1.draw() #self.canvas1.show()
        self.canvas1.get_tk_widget().grid(row=2, column=0, rowspan=5, columnspan=5,sticky='nesw')
         
        self.canvas1.draw() #self.canvas1.show()
        
        #2nd graph on top
        self.canvas2 = FigureCanvasTkAgg(self.fig2,master=master)
        self.canvas2.draw() #self.canvas2.show()
        self.canvas2.get_tk_widget().grid(row=2, column=7, rowspan=5, columnspan=5,sticky='nesw')

        #toolbars for nagivating the graphs
        toolbar_frame1 = Frame(master)
        toolbar_frame1.grid(row=8,column=0, columnspan=5)
        toolbar1 = NavigationToolbar2TkAgg(self.canvas1, toolbar_frame1)

        toolbar_frame2 = Frame(master)
        toolbar_frame2.grid(row=8, column=6, columnspan=5)
        toolbar2 = NavigationToolbar2TkAgg(self.canvas2, toolbar_frame2)
        
        zLabelFrame = Frame(master)
        zLabel = Label(zLabelFrame, text="Z velocity").pack()
        zLabelFrame.grid(row=0,column=0,columnspan=5,sticky='nesw')

        videoLabelFrame = Frame(master)
        videoLabel = Label(videoLabelFrame, text="Live video").pack()
        videoLabelFrame.grid(row=9,column=0,columnspan=5,sticky='nesw')
        
        xLabelFrame = Frame(master)
        xLabel = Label(xLabelFrame, text="Y Linear Acceleration").pack()
        xLabelFrame.grid(row=0,column=7,columnspan=5,sticky='nesw')
        

        #video
        imageFrame = Frame(master,width=self.width)
        imageFrame.grid(row=10, column=0, rowspan=5, columnspan=5, sticky='nesw')         
                
        buttonFrame = Frame(master)
        buttonFrame.grid(row=11, column=7, sticky='nesw')
        timeText = tkinter.StringVar()
        timeText.set("0:00")
        timeLabel = Label(buttonFrame, textvariable=timeText).pack()

        #Capture video frames
        lmain = Label(imageFrame)
        lmain.grid(row=10, column=0, rowspan=5,columnspan=5, sticky='nesw')
        cap = cv2.VideoCapture(video_file)
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        self.current_frame = 0

        success, frame = cap.read()

        self.pause = False
        
        def on_clicked_pause():
            self.pause = True
            #zoom in on graph with 2 second margin
            seconds = self.current_frame*1.0/(fps)
            x = 0
            if(seconds - 2 > 0):
                x = seconds - 2.0
            y = seconds + 2.0
            self.ax.set_xlim([x,y])
            self.ax2.set_xlim([x,y])
            self.ln1.set_data([seconds,seconds], self.ax.get_ylim())
            self.ln2.set_data([seconds,seconds], self.ax2.get_ylim())
            self.canvas1.draw()
            self.canvas2.draw()

        
        def on_clicked_play():
            if(self.pause):
                self.pause = False
                show_frame()

        def on_clicked_back():
            self.current_frame -=fps*5
            if(self.current_frame < 0):
                self.current_frame = 0

        def on_clicked_forward():
            self.current_frame +=fps*5
            if(self.current_frame > frame_count):
                self.current_frame = frame_count
                self.pause = True

        pauseButton = Button(buttonFrame, text="pause", command=on_clicked_pause).pack()
        playButton = Button(buttonFrame, text="play", command=on_clicked_play).pack()
        backButton = Button(buttonFrame, text="back", command=on_clicked_back).pack()
        forwardButton = Button(buttonFrame, text="forward", command=on_clicked_forward).pack()
        
        def onclick(event):
            if event.dblclick:
                x = event.x
                x1, x2 = self.ax.get_xlim()
                size,_ = self.fig.get_size_inches()*self.fig.dpi;
                ratio = x*1.0/size
                time = x1 + (x2-x1)*ratio
                self.current_frame = int(time*fps)
                x1 = time-2
                x2 = time+2
                self.ax.set_xlim([x1,x2])
                self.ax2.set_xlim([x1,x2])
                #update vertical lines
                self.ln1.set_data([time,time], self.ax.get_ylim())
                self.ln2.set_data([time,time], self.ax2.get_ylim())

                
        cid = self.fig.canvas.mpl_connect('button_press_event', onclick)
        #Call this to start updating the plot
        #Works by recursively calling itself after a delay, stopping if you pause (hence the need to restart it when you hit play)
        self.current_frame = 0
        def show_frame():
            #Recursively call this function after some delay
            #First number is that time delay (at minimum; if the rest of this function is slow, it may delay the recursive call further)
            #Should be 1000/fps, not fps.
            #Fps seemed to run in real time experimentally because we were calling this AFTER all the heavy processing. We want to start the clock BEFORE updating images so we don't add an extra delay on top of everything
            lmain.after_id = lmain.after(1000//fps, show_frame)

            #if self.pause: #...except stop recursively calling this if we're paused or at the end of the file
            if self.pause or self.current_frame >= frame_count: #...except stop recursively calling this if we're paused or at the end of the file
                lmain.after_cancel(lmain.after_id)
            
            if self.current_frame >= frame_count:
                cap.set(cv2.CAP_PROP_POS_FRAMES,(frame_count-1))
                success, frame = cap.read()
        
            else:
                cap.set(cv2.CAP_PROP_POS_FRAMES, self.current_frame)
                success,frame = cap.read()
                
            #update current time and graphs
            #This part updates the program's printed clock
            if(self.current_frame % fps == 0): #Fires every second
                if (self.current_frame%(60*fps)//fps//10==0): #"If number of seconds < 10". Equivalent: "if (current + back)//fps % 60 < 10" for readability
                    timeText.set((str((self.current_frame)//(60*fps)) + ":0" + str((self.current_frame)%(60*fps)//fps)))
                else:
                    timeText.set((str((self.current_frame)//(60*fps)) + ":" + str((self.current_frame)%(60*fps)//fps)))
                
            #if zoomed in, update graphs every 6 frames
            if(self.current_frame) % 6 == 0:
                seconds = (self.current_frame)*1.0/fps #Current time in seconds; don't round
                x, y = self.ax.get_xlim() #Axis limits are [x, y]
                if(y-x <= 4.5): #If we're zoomed in
                    x = 0
                    if(seconds - 2 > 0): #Start at 2 seconds ago if that's a non-negative time
                        x = seconds - 2.0
                    y = seconds + 2.0 #End 2 seconds from now
                    self.ax.set_xlim([x,y])
                    self.ax2.set_xlim([x,y])
                    #update vertical lines
                    self.ln1.set_data([seconds,seconds], self.ax.get_ylim())
                    self.ln2.set_data([seconds,seconds], self.ax2.get_ylim())
                    self.canvas1.show()
                    self.canvas2.show()

            #update image on video
            #640x480
            if(self.current_frame < p_length and not points_df['gap'][self.current_frame]):
                lwx = int(points_df['lwrist.x'][self.current_frame])
                lwy = int(points_df['lwrist.y'][self.current_frame])
                rwx = int(points_df['rwrist.x'][self.current_frame])
                rwy = int(points_df['rwrist.y'][self.current_frame])
                nx = int(points_df['neck.x'][self.current_frame])
                ny = int(points_df['neck.y'][self.current_frame])
                lex = int(points_df['lelbow.x'][self.current_frame])
                ley = int(points_df['lelbow.y'][self.current_frame])
                rex = int(points_df['relbow.x'][self.current_frame])
                rey = int(points_df['relbow.y'][self.current_frame])
                lsx = int(points_df['lshoulder.x'][self.current_frame])
                lsy = int(points_df['lshoulder.y'][self.current_frame])
                rsx = int(points_df['rshoulder.x'][self.current_frame])
                rsy = int(points_df['rshoulder.y'][self.current_frame])

                #if(lwx <= 640):
                cv2.line(frame, (lwx,lwy), (lex,ley), (0,255,255),5)
                cv2.line(frame, (rwx,rwy), (rex,rey), (0,255,255),5)
                #if(lex <= 640):
                cv2.line(frame, (lex,ley), (lsx,lsy), (255,0,255),5)
                cv2.line(frame, (rex,rey), (rsx,rsy), (255,0,255),5)
                cv2.line(frame, (lsx,lsy), (nx,ny), (0,128,255),5)
                cv2.line(frame, (rsx,rsy), (nx,ny), (0,128,255),5)

                #if(lwx <= 640):
                cv2.circle(frame, (lwx,lwy), 10, (255,0,0), -1)
                cv2.circle(frame, (rwx,rwy), 10, (255,0,0), -1)
                cv2.circle(frame, (nx,ny), 10, (0,0,255), -1)
                #if(lsx <= 640):
                cv2.circle(frame, (lex,ley), 10, (0,255,0), -1)
                cv2.circle(frame, (rex,rey), 10, (0,255,0), -1)
                cv2.circle(frame, (lsx,lsy), 10, (255,255,0), -1)
                cv2.circle(frame, (rsx,rsy), 10, (255,255,0), -1)
            
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            img = Image.fromarray(cv2image)
            img = img.resize((self.videowidth, self.videoheight))
            imgtk = ImageTk.PhotoImage(image=img)
            lmain.imgtk = imgtk
            lmain.configure(image=imgtk)

            #Stop iterating at end of file
            if self.current_frame >= frame_count:
                self.pause = True
            
            self.current_frame +=1
            
        #Call show_frame once at the end of initialization to start the video playing
        show_frame()


#Call the initialization and start the mainloop (loop forever responding to user input)
root = tkinter.Tk()
root.resizable(width=True, height=True)
width, height = int(root.winfo_screenwidth()/2), root.winfo_screenheight()
root.geometry('%dx%d+0+0' % (width,height))

gyrofile = read_file(sys.argv[1])
gyro_df = get_gyro_df(gyrofile)
video_file = sys.argv[2]
points_df = pd.DataFrame()
if(sys.argv[3] != "novice"):
    openpose_file = sys.argv[3]
    openpose_df = get_openpose_df(openpose_file)
    openpose = get_points(openpose_df)
    points_df = interpolate(openpose)

root2 = tkinter.Toplevel()
root2.resizable(width=True, height=True)
width, height = int(root2.winfo_screenwidth()/2), root2.winfo_screenheight()
root2.geometry('%dx%d+0%d+0' % (width,height,width))

gyrofile2 = read_file(sys.argv[4])
gyro_df2 = get_gyro_df(gyrofile)
video_file2 = sys.argv[5]
points_df2 = pd.DataFrame()
if(sys.argv[6] != "novice"):
    openpose_file2 = sys.argv[6]
    openpose_df2 = get_openpose_df(openpose_file2)
    openpose2 = get_points(openpose_df2)
    points_df2 = interpolate(openpose2)


app = App(root,gyro_df,video_file,points_df)
app2 = App(root2,gyro_df2,video_file2,points_df2)

root.mainloop()

