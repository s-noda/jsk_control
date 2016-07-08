from rosbag import *
from pylab import *
import numpy as np
import traceback
import sys
import pylab
import rospy
import roslib
from sensor_msgs.msg import JointState
import os

params = {'backend': 'ps',
          'axes.labelsize': 20,
          'text.fontsize': 20,
          'legend.fontsize': 20,
          'xtick.labelsize': 15,
          'ytick.labelsize': 15,
          "figure.subplot.right": 0.95,
          "figure.subplot.top": 0.95,
          "figure.subplot.left": 0.05,
          "figure.subplot.bottom": 0.15,
          ##'text.usetex': True,
          'figure.figsize': [8, 5]}
pylab.rcParams.update(params)

class BagGraph:

    def __init__ (self, inpath):
        self.inpath=inpath
        self.outpath="/tmp/output.log"
        self.topics = []
        self.members = []
        self.ids = []
        self.ylabel_name = ''
        self.tmp_label_name=''
        self.title_name = self.inpath
        self.start_time = None
        self.end_time = None
        self.plot_opt = None
        self.time_step = 0
        print ""
        print inpath + " loading ..."
        ##
        self.file_path = None
        ##try:
        self.bag = Bag(inpath)
        ##
        self.fig = pylab.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        ##
        self.js_pub=''
        if (os.system("if [ \"`rosnode list`\" ]; then return 0; else return 1; fi") == 0):
            rospy.init_node("bag_braph_with_joint_states", anonymous=True)
            self.js_pub = rospy.Publisher("/bag_graph/joint_states", JointState)
        else:
            print "roscore not found!!!"
        ##except ROSBagUnindexedException as ex:
        ##print "invalid rosbag file"

    def parse_arg(self, start, argv):
        print "f> parse_arg"
        mode = 'w'
        prev_mode = mode
        members_cnt = -1
        self.file_path = None
        self.topics = []
        self.members = []
        self.ids = []
        self.plot_opt = None
        self.time_step = 0
        print "[options]"
        for i in range(start,len(argv)):
            if ( len(argv[i]) == 0 ):
                print "empty string detected"
            elif ( argv[i][0] == '-' ):
                prev_mode = mode
                mode = argv[i][1]
            elif ( mode == 'w' ):
                print "   " + "skip " + argv[i]
            elif ( mode == 'f' ):
                print "   " + "file mode"
                self.file_path = argv[i]
            elif ( mode == 's' ):
                print "   " + "start_time = " + argv[i]
                try:
                    self.start_time = rospy.Time(float(argv[i]))
                except ValueError:
                    self.start_time = None
            elif ( mode == 'e' ):
                print "   " + "end_time = " + argv[i]
                try:
                    self.end_time = rospy.Time(float(argv[i]))
                except ValueError:
                    self.end_time = None
            elif ( mode == 'r' ):
                print "   " + "time_step = " + argv[i]
                try:
                    self.time_step = float(argv[i])
                except ValueError:
                    self.time_step = 0
            elif ( mode == 'p' ):
                print "   " + "plot_opt = " + argv[i]
                try:
                    self.plot_opt = str(argv[i])
                except ValueError:
                    self.plot_opt = None
            elif ( mode == 'i' ):
                print "   " + "inpath = " + argv[i]
                self.inpath = argv[i]
            elif ( mode == 'o' ):
                print "   " + "outpath = " + argv[i]
                self.outpath = argv[i]
            elif ( mode == 't' ):
                print "   " + "topic add " + argv[i]
                self.topics.append(argv[i])
            elif ( mode == 'l' ):
                print "   " + "tmp label add " + argv[i]
                self.tmp_label_name = argv[i]
            elif ( mode == 'g' ):
                print "   " + "title add " + argv[i]
                self.title_name = argv[i]
            elif ( mode == 'm' ):
                if ( prev_mode != mode ):
                    prev_mode = mode
                    members_cnt = members_cnt + 1
                    self.members.append([])
                print "   " + "member add " + argv[i]
                self.members[members_cnt].append(argv[i])
            elif ( mode == 'y' ):
                print "   " + "ylabel = " + argv[i]
                self.ylabel_name = argv[i]
            elif ( mode == 'n' ):
                print "   " + "number = " + argv[i]
                if ( len(self.ids) <= members_cnt ):
                    self.ids.append([])
                self.ids[members_cnt].append( int(argv[i]) )
            else:
                print "   " + "unknown mode " + mode
                mode = 'w'

    def update_buf_file(self):
        print "f> update buf file"
        self.time_buf = [[]]
        self.val_buf = [[[]]]
        self.label_buf = [[self.tmp_label_name]]
        ##
        fin = open(self.file_path,"r")
        line = fin.readline()
        prev_time = 0
        while line:
            val = float(line.split(" ")[1])
            tm = float(line.split(" ")[0])
            if ( tm - prev_time >= self.time_step and tm >= self.start_time.to_sec() and tm < self.end_time.to_sec() ):
                prev_time = tm
                self.val_buf[0][0].append(val)
                self.time_buf[0].append(tm)
            line = fin.readline()
        fin.close()

    def update_buf(self):
        if self.file_path:
            self.update_buf_file()
            return
        ##
        print "f> update buf"
        self.time_buf = []
        self.val_buf = []
        self.label_buf = []
        first = True
        prev_time = 0
        for i in range(max(len(self.topics), len(self.members), 1)):
            self.val_buf.append([])
            self.time_buf.append([])
            self.label_buf.append([])
        for topic, msg_raw, t in self.bag.read_messages(topics=self.topics, raw=False, start_time=self.start_time, end_time=self.end_time):
            if ( t.to_sec() - prev_time < self.time_step ):
                continue
            prev_time = t.to_sec()
            members_cnt = -1
            while True:
                label_name = ""
                msg = msg_raw
                if ( len(self.topics) != 0 ):
                    try:
                        members_cnt = self.topics.index(str(topic), members_cnt+1)
                    except ValueError:
                        break
                    label_name = label_name + str(self.topics[members_cnt])
                    if ( len(self.members) > members_cnt ):
                        for x in self.members[members_cnt]:
                            label_name = str(x) ##label_name + "/" + str(x)
                            msg = getattr(msg, x)
                self.time_buf[members_cnt].append(t.to_sec())
                if (isinstance(msg, tuple)):
                    msg = list(msg)
                if (isinstance(msg,int) or isinstance(msg,long) or isinstance(msg,float)):
                    msg = [msg]
                if ( not isinstance(msg, list) ):
                    print "invalid graph data"
                    print msg
                    raise Exception()
                if len(self.val_buf[members_cnt]) == 0:
                    id_buf = range(len(msg))
                    if ( len(self.ids) > members_cnt ):
                        if ( isinstance(self.ids[members_cnt],list) ):
                            id_buf = self.ids[members_cnt]
                        else:
                            id_buf = [ self.ids[members_cnt] ]
                    for i in id_buf:
                        if ( len(self.tmp_label_name) > 0 ):
                            self.label_buf[members_cnt].append(self.tmp_label_name)
                            self.tmp_label_name = ''
                        else:
                            ## self.label_buf[members_cnt].append(label_name + "[" + str(i) + "]")
                            self.label_buf[members_cnt].append(None)
                        self.val_buf[members_cnt].append([msg[i]])
                else:
                    tmp_id = 0
                    id_buf = range(len(self.val_buf[members_cnt]))
                    if ( len(self.ids) > members_cnt ):
                        if ( isinstance(self.ids[members_cnt],list) ):
                            id_buf = self.ids[members_cnt]
                        else:
                            id_buf = [ self.ids[members_cnt] ]
                    for i in id_buf:
                        self.val_buf[members_cnt][tmp_id].append(msg[i])
                        tmp_id = tmp_id + 1
                if ( len(self.topics) == 0 ):
                    break

    def pop_line(self):
        self.ax.lines.pop(0)
        self.fig.show()

    def clear_all(self):
        for i in range(len(self.ax.lines)):
            self.ax.lines.pop(0)
        self.fig.show()

    def onclick(self,event):
        data_buf = []
        print ""
        print "time=" + str(event.xdata)
        return
        self.ax.yaxis.set_ticks_position('left')
        self.ax.spines['left'].set_position(('data',event.xdata))
        ylabel(str(self.ylabel_name))
        ## lgd = self.ax.legend(loc="upper left", bbox_to_anchor=(1.01,1.0))
        lgd = self.ax.legend(loc="upper left", prop={'size' : 20})
        self.fig.show()
        for topic, msg_raw, t2 in self.bag.read_messages(topics=self.topics, raw=False, start_time=rospy.Time(event.xdata-0.001), end_time=rospy.Time(event.xdata+0.001)):
            print str(topic) + "=" + str(msg_raw)
        # for i in range( len(self.time_buf) ):
        #     for j in range( len(self.time_buf[i]) ):
        #         if ( self.time_buf[i][j] > event.xdata ):
        #             tmp = []
        #             for k in range( len(self.val_buf[i]) ):
        #                 tmp.append( self.val_buf[i][k][j] )
        #                 print str(self.label_buf[i][k]) + "=" + str(self.val_buf[i][k][j])
        #             data_buf.append( tmp )
        #             break
        ##
        if (self.js_pub):
            for topic, msg_raw, t in self.bag.read_messages(topics="/joint_states", raw=False, start_time=rospy.Time(event.xdata-0.001)):
                if ( t.to_sec() > event.xdata ):
                    self.js_pub.publish(msg_raw)
                    break
        ##
        print ""
        print "ready>"

    def draw_graph(self):
        print "f> draw_graph"
        for i in range(len(self.val_buf)):
            for j in range(len(self.val_buf[i])):
                if self.plot_opt:
                    self.ax.plot(self.time_buf[i], self.val_buf[i][j], self.plot_opt, label=self.label_buf[i][j])
                else:
                    self.ax.plot(self.time_buf[i], self.val_buf[i][j], label=self.label_buf[i][j])
        title(str(self.title_name))
        xlabel('time [sec]', fontsize=20)
        ylabel(str(self.ylabel_name) , fontsize=20)
        ## lgd = self.ax.legend(loc="upper left", bbox_to_anchor=(1.01,1.0))
        lgd = self.ax.legend(loc="upper left", prop={'size' : 20})
        self.fig.show()
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

if __name__ == '__main__':
    bag_path = "bag.bag"
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    bg = BagGraph(bag_path)
    print ""
    while True:
        print "ready>"
        command = raw_input()
        if len(command) == 0:
            print "[info] empty skip"
            continue
        elif command[0] == 'q':
            print "[info] quit"
            break
        elif command[0] == 'r':
            print "[info] remove"
            bg.pop_line()
        elif command[0] == 'c':
            print "[info] clear all"
            bg.clear_all()
        elif command[0] == 's':
            print "[info] sample plot"
            bg.parse_arg(0,["-t", "/joint_states", "-m", "position"])
            bg.update_buf()
            bg.draw_graph()
        else:
            print "[info] plot"
            try:
                bg.parse_arg(0,command.split(" "))
                bg.update_buf()
                bg.draw_graph()
            except Exception, exp:
                traceback.print_exc()
                print "invalid input " + str(command)
    print "[info] done"
## python plotbag.py -i ../staro/bag/wall-standup.bag -t /joint_states -m position
