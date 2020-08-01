import math
import pygame as pg
import socket
import threading
import time

### MESSAGE TYPES
## BOT
# login: l;user;pwd --> l;minibots;aws@123
# ping: i;ip;port
# telemetry: t;ts;roll;yaw;pitch;accelWorldX;accelWorldY;accelWorldZ;voltage;heap
# debug: d;message
#
## SERVER->BOT
# ping received: r;;
# calibration: o;botId;mpuInit;leftFreq;leftPeriod;leftStopped;leftForward;leftBackward;rightFreq;rightPeriod;rightStopped;rightForward;rightBackward;speed1;speed2;speed3
# position: p;x;y
# direction: d;x;y --> unitary
# seek: s;x;y
# stop: t;;
#
## SERVER->TRAFFIC LIGHT (botid 1000)
# t;b;b;b;b;b;b;b;b --> b=1|0  0=red 1=green

# TODO: parameter
SERVER_IP='localhost'
SERVER_PORT=5000

class Bot(object):
    def __init__(self, color="red", id=-1, tag=None, position=(0,0), angle=0, host=None, port=None, debug=False):
        if host is None or port is None or tag is None or id == -1:
            raise Exception( "You need to inform: host[%s] port[%s] tag[%s] id[%s]" % (host, port, tag, id))
        self.debug = debug
        self.id = id
        self.speed = [3, 4, 5]

        self.image = pg.image.load("assets/%s_car.png" % color)
        self.image.blit(tag, (2,8))
        self.image.convert()
        self.ref_image = self.image.copy()
        self.rect = self.ref_image.get_rect()

        self.offset = pg.math.Vector2(15, 0)
        self.rotated_offset = pg.math.Vector2(self.offset)
        self.pos = pg.math.Vector2(position)
        self.dir = pg.math.Vector2(1,0)
        self.pivot = self.image.get_rect().center
        self.seek = None

        self.parking_pos = pg.math.Vector2(self.pos)
        self.parking_angle = angle

        self.rotate(angle)
        self.stop()

        self.last_ping = 0
        self.connected = False
        self.active = False
        self.running = True

        self.dragging = False
        self.dragging_offset = None

        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.sock.bind((host, port))
        self.message_thread = threading.Thread(target=self.process_message, args=())
        self.message_thread.start()

    def park(self):
        self.pos = pg.math.Vector2(self.parking_pos)
        self.dir = pg.math.Vector2(1, 0)
        self.rotate(self.parking_angle)
        self.stop()
        self.__update_body__()

    def stop(self):
        self.seek = self.pos + self.pivot

    def send_message(self, msg):
        self.sock.sendto( msg.encode('utf-8'), (SERVER_IP, SERVER_PORT))

    def kill(self):
        self.running = False
        self.sock.close()
        self.message_thread.join()

    def update(self, screen):
        self.blit(screen)

        if not self.active:
            return

        if not self.connected and pg.time.get_ticks() - self.last_ping   > 2000:
            print("pinging server..")
            self.send_message("i;%s;%d;%d" % (self.host, self.port, self.id))
            self.last_ping = pg.time.get_ticks()
            return

        nextDir = self.seek - (self.pos + self.pivot)
        distToTarget = nextDir.magnitude()

        if distToTarget > 1:
            cross = nextDir.cross(self.dir)
            deltaAngle = math.atan2(cross, nextDir.dot(self.dir))
            absDeltaAngle = abs(deltaAngle)

            if absDeltaAngle > 0.1: # step 1
                step = self.speed[1] if absDeltaAngle > math.pi/2 else self.speed[0]
                if deltaAngle >= 0: # turn left
                    self.rotate(-step)
                else: # turn right
                    self.rotate(step)
            else: # go straight
                step = self.speed[2] if distToTarget > 30 else self.speed[0]
                self.move(step/5)
 
    def __update_body__(self):
        self.rect = self.image.get_rect(center=self.pos + self.pivot + self.rotated_offset)

    def move(self, step):
        self.pos += self.dir * step
        self.__update_body__()

    def rotate(self, angle):
        self.dir = self.dir.rotate(angle)
        angle = pg.math.Vector2(1,0).angle_to(self.dir)

        self.image = pg.transform.rotate(self.ref_image, -angle)
        self.rotated_offset = self.offset.rotate(angle)
        self.__update_body__()

    def blit(self, screen):
        screen.blit(self.image, self.rect )
        if self.debug:
            pg.draw.rect(screen, (0,0,255), self.rect, 2)
            pg.draw.circle(screen, (255, 0, 255), self.seek, 30, 1)

    def process_message(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                #print("received message: %s %s" % (data, addr))
                data = data.decode('utf-8')
                tokens = data.split(';')
                if tokens[0] == 'r':
                    self.connected = True
                    self.send_message("l;minibots;aws@123;%d" % self.id)
                elif tokens[0] == 'o':  # config
                    _, botId, mpuInit, leftFreq, leftPeriod, leftStopped, leftForward, leftBackward, rightFreq, rightPeriod, rightStopped, rightForward, rightBackward, speed1, speed2, speed3 = tokens
                elif tokens[0] == 'p':  # position
                    # self.pos = pg.math.Vector2(float(tokens[1]), float(tokens[2]))
                    pass
                elif tokens[0] == 'd':  # direction
                    # self.cam_dir = pg.math.Vector2(float(tokens[1]), float(tokens[2]))
                    pass

                elif tokens[0] == 's':  # seek point
                    self.seek = pg.math.Vector2(float(tokens[1]), float(tokens[2]))

                elif tokens[0] == 't':  # stop!
                    self.stop()
                else:
                    print(tokens)
            except OSError as e:
                if e.errno != 9:
                    print(e)