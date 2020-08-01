#!/usr/local/bin/python3
import socket
import numpy as np
import cv2
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192*2 )
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

backend_ip = '127.0.0.1'
backend_port = 9991
sock.bind((backend_ip, backend_port))

tick = time.time()

while True:
    frame = sock.recv(8192)
    frame = np.fromstring(frame, dtype=np.uint8)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    fps = (time.time() - tick) * 1000
    cv2.putText(frame, "FPS: %04d" % fps, (50,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.LINE_AA)

    cv2.imshow("frame", frame)
    if cv2.waitKey(20) == ord('q'):
        break

    tick = time.time()
