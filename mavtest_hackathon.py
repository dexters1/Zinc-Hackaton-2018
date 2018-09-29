#!/usr/bin/env python

import sys, os, time
from optparse import OptionParser

# tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '/mnt/c/Users/marce/DEV/mavlink'))
from pymavlink import mavutil
from pymavlink.dialects.v20 import aerotain as at
from goprocam import GoProCamera
from goprocam import constants
import cv2
import sys
 
(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')



def handle_heartbeat(msg):
    mode = mavutil.mode_string_v10(msg)
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED


def handle_attitude(msg):
    attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed,
                     msg.pitchspeed, msg.yawspeed)
    #print ("Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd")
    #print ("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data)


def loop(m):
    #definisanje promenljivih pre petlje
    mav = at.MAVLink(m)

    xaxis_output = 0
    yaxis_output = 0
    zaxis_output = 0
    xraxis_output = 0
    yraxis_output = 0
    zraxis_output = 0
    counter = 0
	
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN'] #Biramo algoritam za pracenje objekta, a objekat pravimo pomocu BoundingBox-a
    tracker_type = tracker_types[2]
 
    if int(minor_ver) < 3:
        tracker = cv2.Tracker_create(tracker_type) 
    else:
        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv2.TrackerGOTURN_create()
 
    # Read video
    video = cv2.VideoCapture("udp://127.0.0.1:10000") #cita se video iz memorije, prethodno mora biti pokrenut gopro fajl da cuva lajv strim na toj memoriskoj lokaciji
   
 
    # Exit if video not opened.
    if not video.isOpened():
        print ("Could not open video")
        #sys.exit()
 
    # Read first frame.
    ok, frame = video.read()
    if not ok:
        print ('Cannot read video file')
        sys.exit()
     
    # Define an initial bounding box
    bbox = (287, 23, 86, 320)
 
    # Uncomment the line below to select a different bounding box
    bbox = cv2.selectROI(frame, False)
 
    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)   
    #end of new part
    a = 0.5
    xaxis_output = 0
    yaxis_output = 0
    zaxis_output = 0
    while (True):
        #new part
        # Read a new frame
        xaxis_outputNEW = 0
        yaxis_outputNEW = 0
        zaxis_outputNEW = 0
        xraxis_output = 0
        yraxis_output = 0
        zraxis_output = 0
        ok, frame = video.read()
        if not ok: # U slucaju da ima problem sa citanjem lajv strima da preskoci iteraciju obrade i pokusa opet
            continue    
         
        # Start timer
        timer = cv2.getTickCount()
 
        # Update tracker
        ok, bbox = tracker.update(frame)
 
        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
 
        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
 
        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
     
        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
 
        # Display result
        cv2.imshow("Tracking", frame)
 
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff #provera tastera pritisnutog na tastaturi
        if k == 27 : break
        if k == 112:  #Menjamo Bounding Box i biramo novi objekat sa screenshot-a lajv strima koji zelimo da pratimo
           if tracker_type == 'BOOSTING':
                tracker = cv2.TrackerBoosting_create()
           if tracker_type == 'MIL':
                tracker = cv2.TrackerMIL_create()
           if tracker_type == 'KCF':
                tracker = cv2.TrackerKCF_create()
           if tracker_type == 'TLD':
                tracker = cv2.TrackerTLD_create()
           if tracker_type == 'MEDIANFLOW':
                tracker = cv2.TrackerMedianFlow_create()
           if tracker_type == 'GOTURN':
                tracker = cv2.TrackerGOTURN_create()
           ok, frame = video.read()
           bbox = cv2.selectROI(frame, False)
           ok = tracker.init(frame, bbox)
           ok, bbox = tracker.update(frame)
        #end of new part

        # grab a mavlink message
        msg = m.recv_match(blocking=False)
        if not msg:
            #discard this message
            continue

        # handle the message based on its type
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        elif msg_type == "HEARTBEAT":
            handle_heartbeat(msg)
        elif msg_type == "ATTITUDE":
            handle_attitude(msg)

            # if msg.roll > 0 and xaxis_output < 32757:
                # xaxis_output += 10
            # elif msg.roll < 0 and xaxis_output > -32757:
                # xaxis_output -= 10
            
            if k == 119: #forward picture_mode || xaxis_output = 32757
                xaxis_outputNEW = 32757
            if k == 115: #backwards
                xaxis_outputNEW = -32757
            if k == 97:  #left
                yaxis_outputNEW = 32757
            if k == 100: #right
                yaxis_outputNEW = -32757
            if k == 113: #up
                zaxis_outputNEW = -32757
            if k == 101: #down
                zaxis_outputNEW = 32757
            if k == 108: #take a picture
                ok, frame = video.read()
                cv2.imwrite("frame%d.jpg" % counter, frame)
            print(bbox)
            if (bbox[0] + (bbox[2]/2)) > 210: #gledamo da li je BoundingBox previse desno i u slucaju da jeste kazemo dronu da ide brze desno da odrzi boundingbox u centru
                print('too far right')
                zraxis_output = 12000
                # zraxis_output = -(((bbox[0] - 240)/160)*(-32760)) Pokusaj skaliranja koji nije bio optimizovan da dron sto je BoundingBox dalje od centra da ga jacim intenzitetom vraca ka centru
                # if zraxis_output < -32760 : 
                    # zraxis_output = -32760
                # if zraxis_output > 0 : 
                    # zraxis_output = 0
            
            elif (bbox[0] + (bbox[2]/2)) < 190:
                print('too far left')
                zraxis_output = -12000
                # zraxis_output = -(32760 - (32760*(bbox[0]))/160)
                # if zraxis_output > 32760 :
                    # zraxis_output = 32760
                # if zraxis_output < 0 :
                    # zraxis_output = 0

            if (bbox[1] + (bbox[3]/2)) > 110:
                print('too far down')
                yraxis_output = -8000
                # yraxis_output = 32760 - 32760*(200-bbox[1])/50 #check the sign
                # if yraxis_output > 32760 :
                    # yraxis_output = 32760
                # if yraxis_output < 0 :
                    # yraxis_output = 0
            elif (bbox[1] + (bbox[3]/2)) < 90:
                print('too far up')
                yraxis_output = 8000
                # yraxis_output = -32760 + (bbox[1]*32760)/50 #check the sign
                # if yraxis_output < -32760 :
                    # yraxis_output = -32760
                # if yraxis_output > 0 :
                    # yraxis_output = 0

            if bbox[1] == 0: #Ako nema nikakav BoundingBox ili ako se izgubi da ne radi nista
                yraxis_output = 0
            if bbox[0] == 0:
                zraxis_output = 0
            
			 
            xaxis_output = xaxis_output * a + xaxis_outputNEW *(1-a) #Skaliranje kretanja drona na komande tako da sto duze drzis dugme sto se vise dron ubrzava
            yaxis_output = yaxis_output * a + yaxis_outputNEW *(1-a)
            zaxis_output = zaxis_output * a + zaxis_outputNEW *(1-a)
            
            # print("y axis: ", yraxis_output)
            print("z axis: ", zraxis_output)
            # print()
            
            mav.setpoint_6dof_send(int(xaxis_output), int(yaxis_output), int(zaxis_output), int(xraxis_output), int(yraxis_output), int(zraxis_output), counter)
            counter += 1

def main():
    # read command line options
    parser = OptionParser("readdata.py [options]")
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="master port baud rate", default=115200)
    parser.add_option("--device", dest="device", default= "\\.\COM4", help="serial device")
    parser.add_option("--rate", dest="rate", default=4, type='int', help="requested stream rate")
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=255, help='MAVLink source system for this GCS')
    parser.add_option("--showmessages", dest="showmessages", action='store_true',
                      help="show incoming messages", default=True)
    (opts, args) = parser.parse_args()

    if opts.device is None:
        print("You must specify a serial device")
        sys.exit(1)

    # create a mavlink serial instance
    master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

    # wait for the heartbeat msg to find the system ID
    master.wait_heartbeat()

    # request data to be sent at the given rate
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)


    # enter the data loop
    loop(master)


if __name__ == '__main__':
    main()