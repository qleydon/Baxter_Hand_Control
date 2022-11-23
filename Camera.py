import mediapipe as mp
import cv2
import pyrealsense2
import numpy as np
import math
import asyncio
import socket
import sys
from scipy.spatial.transform import Rotation as R

# --------------------------------------------------------------------------------------------------------------
# Choose Z type
# 0 = Constant depth
# 1 = MediaPipe depth (Beta)
z_type = 1
Cz = 0.8
# --------------------------------------------------------------------------------------------------------------
# Chose Camera Orientation:
# 0 = as the camera sees
# 1 = Baxter camera in front
CO = 1
# --------------------------------------------------------------------------------------------------------------
# Chose Orientation type - Baxter can take roll or yaw, not both
# 0 = No Yaw
# 1 = No Roll
RPY_type = 1
# --------------------------------------------------------------------------------------------------------------
# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('127.0.0.1', 10001)
print(sys.stderr, 'starting up on %s port %s' % server_address)
#sock.bind(server_address)
print(sys.stderr, '\nwaiting to receive message')
sock.bind(('0.0.0.0', 10001))
data_message, address = sock.recvfrom(4096)
# --------------------------------------------------------------------------------------------------------------
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0)
last_image = 0

P1 = [0, 0, Cz]
P2 = [0, 0, Cz]
P3 = [0, 0, Cz]
pos = [0, 0, Cz]
# --------------------------------------------------------------------------------------------------------------
async def main():
    with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5) as hands:
        while cap.isOpened():
            ret, frame = cap.read()

            # BGR 2 RGB
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            last_image = image
            # Flip on horizontal
            image = cv2.flip(image, 1)
            height, width, channels = image.shape
            # Set flag
            image.flags.writeable = False

            # Detections
            results = hands.process(image)

            # Set flag to true
            image.flags.writeable = True

            # RGB 2 BGR
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # --------------------------------------------------------------------------------------------------------------
            if results.multi_hand_landmarks:
                for num, hand in enumerate(results.multi_hand_landmarks):
                    mp_drawing.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS,
                                              mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4),
                                              mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2),
                                              )
                wrist = results.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.WRIST]
                if (wrist.x * width != 0): P1[0] = wrist.x# * width
                if (wrist.y * height != 0): P1[1] = wrist.y# * height
                if z_type == 1:
                    P1[2] = -(wrist.z * 2)

                index_MCP = results.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
                if (index_MCP.x * width != 0): P2[0] = index_MCP.x #* width
                if (index_MCP.y * height != 0): P2[1] = index_MCP.y #* height
                if z_type == 1:
                    if index_MCP.z != 0: P2[2] = -(index_MCP.z * 2)

                p_MCP = results.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.PINKY_MCP]
                if (p_MCP.x * width != 0): P3[0] = p_MCP.x #* width
                if (p_MCP.y * height != 0): P3[1] = p_MCP.y #* height
                if z_type == 1:
                    if p_MCP.z != 0: P3[2] = -(p_MCP.z * 2)

            cv2.imshow('Hand Tracking', image)
            # --------------------------------------------------------------------------------------------------------------
            V1 = [P2[0] - P3[0], P2[1] - P3[1], P2[2] - P3[2]] #Vector from pinky to index
            V2 = [P1[0] - P3[0], P1[1] - P3[1], P1[2] - P3[2]] #vector from pinky to palm

            a = np.cross(V2, V1) #vector out of palm
            # Normalize
            aa = abs(a)
            La = math.sqrt(aa[0] ** 2 + aa[1] ** 2 + aa[2] ** 2)  # find length

            if La == 0: La = 0.1
            a[0] = a[0] / La
            a[1] = a[1] / La
            a[2] = a[2] / La

            o = np.array(V1) #vector from pinky to index
            # Normalize
            ao = abs(o)
            Lo = math.sqrt(ao[0] ** 2 + ao[1] ** 2 + ao[2] ** 2)  # find length
            if Lo == 0: Lo = 0.1
            o[0] = o[0] / Lo
            o[1] = o[1] / Lo
            o[2] = o[2] / Lo

            n = np.cross(o, a) # vecor from pinky down
            mat = [n[0], o[0], a[0]], [n[1], o[1], a[1]], [n[2], o[2], a[2]]
            eul = 0

            r = R.from_matrix([[n[0], o[0], a[0]],
                               [n[1], o[1], a[1]],
                               [n[2], o[2], a[2]]])

            # ----------------------------------------------------------------------------------------------------------
            if CO == 0:
                pos[0] = P1[0]
                pos[1] = P1[1]
                eul = r.as_euler('xyz', degrees=True)
                eul[2] = eul[2] + eul[1]
                q = r.as_quat()
            # ----------------------------------------------------------------------------------------------------------
            if CO == 1:  # camera in front

                pos[0] = Cz  # for the realsence get distance to this point and update z. should work great!!!!!!
                pos[1] = P1[0]  # / width
                pos[2] = 1 - P1[1]  # / height

                eul = r.as_euler('xyz', degrees=True)
                tmp = eul
                tmp0 = eul[0]
                eul[0] = 90 - (tmp[2] + tmp[1])
                eul[1] = tmp[1] + 90
                eul[2] = tmp0
                if (eul[2] > -40 and eul[2] < 40): eul[2] = 0
                r = R.from_euler('xyz', eul, degrees=True)
                q = r.as_quat()

            pos_s = str(list(pos))
            q_s = str(list(q))
            if RPY_type == 0:
                message = '0' + ',' + str(pos[0]) + ',' + str(pos[1]) + ',' + str(pos[2]) + ',' + str(
                    eul[0]) + ',' + str(eul[1]) + ',' + str(eul[2]) + ',' + '0'
            if RPY_type == 1:
                message = '0'+','+str(pos[0])+','+str(pos[1])+','+str(pos[2])+','+str(q[0])+','+str(q[1])+','+str(q[2])+','+str(q[3])+','+'0'

            print(eul)

            # --------------------------------------------------------------------------------------------------------------
            task = asyncio.create_task(update_UDP(message))
            await asyncio.sleep(0.05)
            # --------------------------------------------------------------------------------------------------------------

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
# --------------------------------------------------------------------------------------------------------------
async def update_UDP(mes):
    await asyncio.sleep(1)
    if data_message:
        pos_s = mes.encode()
        sent = sock.sendto(pos_s, address)
asyncio.run(main())