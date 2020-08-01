import pygame as pg
import socket
import threading

# TODO: parameter
SERVER_IP='localhost'
SERVER_PORT=5000

class TrafficLight(object):
    def __init__(self, pos, angle):
        self.right_area = ((11,8),(24,25))
        self.forward_area = ((36,8), (30,25))
        self.angle = angle
        self.pos = pos
        self.case = pg.image.load("assets/trafficLight.png")
        self.lights = pg.Surface(self.case.get_size())
        self.status = {'r': 2, 'f': 2} # 1 = red, 2 = green, 3 = yellow
        self.__update__()

    def blit(self, screen):
        screen.blit(self.body, self.rect)

    def __update__(self):
        body = self.lights.copy()
        for k in self.status.keys():
            area = self.right_area if k == "r" else self.forward_area
            color = (255, 255, 0)
            if self.status[k] == 1:
                color = (0, 255, 0)
            elif self.status[k] == 2:
                color = (255, 255, 0) # yellow
            body.fill( color, rect=area)
        body.blit(self.case, (0, 0))
        body = pg.transform.rotate(body, self.angle)
        self.body,self.rect = body, body.get_rect().move(self.pos)

    def set_status(self, dir, status):
        if dir not in [ 'r', 'f'] or status < 0 or status > 2:
            raise Exception( "Invalid values for the Traffic light status: dir[%s] status[%d]" % (dir, status))
        self.status[dir] = status
        self.__update__()

class TrafficLightManager(object):
    def __init__(self, id=1000, host='127.0.0.1', port=7001):

        self.host = host
        self.port = port
        self.id = id
        self.running = True
        self.connected = False
        self.last_ping = 0

        self.traffic_lights = [
            TrafficLight((343, 123), 0),
            TrafficLight((479, 146), 90),
            TrafficLight((116, 266), -90),
            TrafficLight((222, 317), 180)
        ]

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.sock.bind((self.host, self.port))
        self.message_thread = threading.Thread(target=self.process_message, args=())
        self.message_thread.start()

    def send_message(self, msg):
        self.sock.sendto( msg.encode('utf-8'), (SERVER_IP, SERVER_PORT))

    def kill(self):
        self.running = False
        self.sock.close()
        self.message_thread.join()

    def update(self, screen):
        for t in self.traffic_lights:
            t.blit(screen)

        if not self.connected and pg.time.get_ticks() - self.last_ping > 2000:
            print("pinging server..")
            self.send_message("i;%s;%d;%d" % (self.host, self.port, self.id))
            self.last_ping = pg.time.get_ticks()
            return

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
                elif tokens[0] == 't':  # traffic lights status
                    tf_id = 0
                    for i in range(8):
                        op = 'f' if i%2 == 0 else 'r'
                        self.traffic_lights[tf_id].set_status(op, int(tokens[i+1]))
                        if i%2 == 1: tf_id += 1

            except OSError as e:
                if e.errno != 9:
                    print(e)