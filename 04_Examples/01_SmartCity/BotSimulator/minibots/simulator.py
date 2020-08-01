import socket
import pygame as pg
import minibots
import numpy as np
import cv2

class Simulator(object):
    def __init__(self, max_fps = 25):

        self.max_fps = max_fps
        # TODO: put this info in a file?
        ## Create an UDP socket to keep sending the frames
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 24)
        self.backend_ip = '127.0.0.1'
        self.backend_port = 9991

        self.bots_host = '127.0.0.1'
        self.bots_port_counter = 4000
        self.bots = {}

        ## Load all the april tags from the mosaic
        self.tags = {}
        tag_mosaic = pg.image.load("assets/mosaic.png")
        for i in range(45 * 47):
            x, y = (i % 45) * 10, (i // 45) * 10
            w, h = x + 9, y + 9
            self.tags[i] = pg.Surface((11, 11))
            self.tags[i].fill((255, 255, 255))
            self.tags[i].blit(tag_mosaic, (1, 1), (x, y, w, h))
            self.tags[i] = pg.transform.rotate(pg.transform.scale(self.tags[i], (23, 23)), -90)

        self.track_area = pg.Rect(10,10,630,470)
        self.panel = pg.Surface((640+200,480))
        self.panel.fill((190, 191, 190))
        self.panel.blit(pg.image.load("assets/track.png"), (0, 0))
        self.panel.blit(pg.image.load("assets/parkinglot.png"), (640, 0))

        arrow = pg.image.load("assets/traffic_sign_arrow.png")
        turn = pg.transform.scale(pg.image.load("assets/traffic_sign_right_turn_prohibited.png"), (30,30))

        # render traffig signs
        self.panel.blit(pg.transform.rotate(turn, 90), (260,130))        
        self.panel.blit(pg.transform.rotate(turn, -90), (347,323))        
        self.panel.blit(pg.transform.rotate(turn, 180), (485,267))
        self.panel.blit(pg.transform.rotate(turn, 0), (123,183))  # 370

        # render traffic direction arrows
        self.panel.blit(arrow, (180, 90))
        self.panel.blit(arrow, (400, 90))
        self.panel.blit(pg.transform.rotate(arrow, 180), (180, 380))
        self.panel.blit(pg.transform.rotate(arrow, 180), (400, 380))

        self.panel.blit(pg.transform.rotate(arrow, 180), (200, 185))
        self.panel.blit(pg.transform.rotate(arrow, 180), (370, 185))
        self.panel.blit(arrow, (200, 280))
        self.panel.blit(arrow, (370, 280))

        self.panel.blit(pg.transform.rotate(arrow, 90), (85, 135))
        self.panel.blit(pg.transform.rotate(arrow, 90), (85, 280))
        self.panel.blit(pg.transform.rotate(arrow, -90), (540, 135))
        self.panel.blit(pg.transform.rotate(arrow, -90), (540, 280))

        # put the traffic lights
        self.traffic_light_manager = minibots.TrafficLightManager()


        self.clock = pg.time.Clock()

        ## Add places
        s = 30
        for i in range(5):
            self.panel.blit(self.tags[20+i], (s, 30))
            self.panel.blit(self.tags[25+i], (s, 480 - 53))
            s += 138
        self.panel.blit(self.tags[30], (30, 229))
        self.panel.blit(self.tags[31], (30 + (138 * 4), 229))

        ## Manual Targets
        self.manual_targets = {}
        for i in range(2):
            self.manual_targets[i] = minibots.ManualTarget(40+i, self.tags[40+i])
            self.manual_targets[i].move((780, 30+(i*50)))

        ## Obstacles
        self.obstacles = {}
        for i in range(4):
            self.obstacles[i] = minibots.Obstacle(35 + i, self.tags[35 + i])
            self.obstacles[i].move((780, 250 + (i * 50)))

        self.dragging_element_id = -1

    def update(self, screen):
        self.clock.tick(self.max_fps)
        screen.blit(self.panel, self.panel.get_rect())

        for o in self.obstacles.values():
            o.blit(screen)

        for m in self.manual_targets.values():
            m.blit(screen)

        for b in self.bots.values():
            b.update(screen)

        self.traffic_light_manager.update(screen)

        img = pg.image.tostring(screen, "RGB")
        img = np.frombuffer(img, dtype=np.uint8)
        img = img.reshape((480, 640 + 200, 3))
        img = img[:, 0:640]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, img = cv2.imencode('.PNG', img)
        try:
            self.sock.sendto(img, (self.backend_ip, self.backend_port))
        except OSError as e:
            _, img_jpg = cv2.imencode('.JPG', img)
            print("Error: %s - image size: png[%d] jpg[%d]" % (e, len(img), len(img_jpg))  )


    def add_bot(self, color, id, debug):
        self.bots[id] = minibots.Bot(
            color, id, self.tags[id], pg.math.Vector2(670, 47+(len(self.bots)*50)), -26,
            self.bots_host, self.bots_port_counter, debug
        )
        self.bots_port_counter += 1

    def shutdown(self):
        for b in self.bots.values():
            b.kill()
        self.traffic_light_manager.kill()

    def stop_dragging(self):
        self.dragging_element_id = -1
        for b in self.bots.values():
            if b.dragging:
                if not self.track_area.collidepoint(b.pos):
                    b.park()
                b.dragging = False

    def drag(self, pos):

        for b in self.bots.values():
            if self.dragging_element_id != -1 and self.dragging_element_id != b.id:
                continue
            if b.rect.collidepoint(pos):
                if not b.dragging:
                    self.dragging_element_id = b.id
                    b.dragging = True
                    b.dragging_offset = pg.math.Vector2(pos) - b.pos
                    continue
                b.pos = pos - b.dragging_offset
                b.__update_body__()
                b.active = self.track_area.collidepoint(b.pos)
                b.stop()
                break

        for m in self.manual_targets.values():
            if self.dragging_element_id != -1 and self.dragging_element_id != m.id:
                continue
            if m.rect.collidepoint(pos):
                if not m.dragging:
                    self.dragging_element_id = m.id
                    m.dragging = True
                    m.dragging_offset = pg.math.Vector2(pos) - m.rect.center
                    continue
                m.rect.move_ip( -(m.rect.center - pg.math.Vector2(pos)))

        for o in self.obstacles.values():
            if self.dragging_element_id != -1 and self.dragging_element_id != o.id:
                continue
            if o.rect.collidepoint(pos):
                if not o.dragging:
                    self.dragging_element_id = o.id
                    o.dragging = True
                    o.dragging_offset = pg.math.Vector2(pos) - o.rect.center
                    continue
                o.rect.move_ip( -(o.rect.center - pg.math.Vector2(pos)))
