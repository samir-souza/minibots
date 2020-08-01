#!/usr/local/bin/python3

import pygame as pg
import minibots

debug = False
if __name__ == "__main__":
    pg.init()
    screen = pg.display.set_mode((640+200, 480))
    black = (0, 0, 0)
    sim = minibots.Simulator()
    for i in range(8):
        sim.add_bot("red" if i%2 == 0 else "blue", i, debug)
    dragging = False
    while True:
        key = pg.key.get_pressed()
        mouse = pg.mouse.get_pressed()
        if key[pg.K_q]: break
        if mouse[0] == True:
            sim.drag( pg.mouse.get_pos() )
            dragging = True
        elif dragging:
            dragging = False
            sim.stop_dragging()

        for event in pg.event.get():
            if event.type == pg.QUIT: break

        screen.fill(black)  # clean the screen
        sim.update(screen)
        pg.display.flip()

    sim.shutdown()
    print("Have a good day! :)")
