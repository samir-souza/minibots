import pygame as pg

class ManualTarget(object):
    def __init__(self, id, tag):
        self.id = id
        self.tag = tag
        self.body = pg.Surface((27, 27))
        self.body.fill((255, 0, 0))
        self.body.blit(tag, (2, 2))
        self.rect = self.body.get_rect()
        self.dragging = False
        self.dragging_offset = None

    def blit(self, screen):
        screen.blit(self.body, self.rect)

    def move(self, pos):
        self.rect = self.rect.move(pg.math.Vector2(pos))