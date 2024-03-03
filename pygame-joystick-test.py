#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vi:ts=4 sw=4 et
from __future__ import division
from __future__ import print_function

import os
os.environ["SDL_VIDEO_ALLOW_SCREENSAVER"] = "1"
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
os.environ["SDL_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR"] = "0"

import sys
import pygame
from pygame.locals import *

import time
import payload
import socket
HOST = '10.110.131.165'
PORT = 38827
format = "utf-8"

remoteControl = True

class joystick_handler(object):
    def __init__(self, id):
        self.id = id
        self.joy = pygame.joystick.Joystick(id)
        self.name = self.joy.get_name()
        self.joy.init()
        self.numaxes    = self.joy.get_numaxes()
        self.numballs   = self.joy.get_numballs()
        self.numbuttons = self.joy.get_numbuttons()
        self.numhats    = self.joy.get_numhats()

        self.axis = []
        for i in range(self.numaxes):
            self.axis.append(self.joy.get_axis(i))

        self.ball = []
        for i in range(self.numballs):
            self.ball.append(self.joy.get_ball(i))

        self.button = []
        for i in range(self.numbuttons):
            self.button.append(self.joy.get_button(i))

        self.hat = []
        for i in range(self.numhats):
            self.hat.append(self.joy.get_hat(i))

class input_test(object):
    class program:
        "Program metadata"
        name    = "Pygame Joystick Test"

    class default:
        "Program constants"
        fontnames = [
            # Bold, Italic, Font name
            (0, 0, "Bitstream Vera Sans Mono"),
            (0, 0, "DejaVu Sans Mono"),
            (0, 0, "Inconsolata"),
            (0, 0, "LucidaTypewriter"),
            (0, 0, "Lucida Typewriter"),
            (0, 0, "Terminus"),
            (0, 0, "Luxi Mono"),
            (1, 0, "Monospace"),
            (1, 0, "Courier New"),
            (1, 0, "Courier"),
        ]
        # TODO: Add a command-line parameter to change the size.
        # TODO: Maybe make this program flexible, let the window height define
        #       the actual font/circle size.
        fontsize     = 20
        circleheight = 10
        resolution   = (640, 480)

    
    def init(self):
        pygame.init()
        pygame.event.set_blocked((MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN))

        self.joycount = pygame.joystick.get_count()
        if self.joycount == 0:
            print("This program only works with at least one joystick plugged in. No joysticks were detected.")
            self.quit(1)

        self.joy = []
        for i in range(self.joycount):
            self.joy.append(joystick_handler(i))

        if remoteControl:
            self.clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.clientSocket.connect((HOST, PORT))


    def run(self):
        self.screen = pygame.display.set_mode((300, 300))
        packet = None

        while True:
            packet = payload.payload_handler()

            pygame.display.flip()
            # self.clock.tick(30)
            for event in [pygame.event.wait(), ] + pygame.event.get():
                packet = payload.payload_handler()
                # QUIT             none
                # ACTIVEEVENT      gain, state
                # KEYDOWN          unicode, key, mod
                # KEYUP            key, mod
                # MOUSEMOTION      pos, rel, buttons
                # MOUSEBUTTONUP    pos, button
                # MOUSEBUTTONDOWN  pos, button
                # JOYAXISMOTION    joy, axis, value
                # JOYBALLMOTION    joy, ball, rel
                # JOYHATMOTION     joy, hat, value
                # JOYBUTTONUP      joy, button
                # JOYBUTTONDOWN    joy, button
                # VIDEORESIZE      size, w, h
                # VIDEOEXPOSE      none
                # USEREVENT        code
                if event.type == QUIT:
                    self.quit()
                elif event.type == KEYDOWN and event.key in [K_ESCAPE, K_q]:
                    self.quit()
                elif event.type == VIDEORESIZE:
                    #self.screen = pygame.display.set_mode(event.size, RESIZABLE)
                    print('TODO')
                elif event.type == JOYAXISMOTION:
                    self.joy[event.joy].axis[event.axis] = event.value
                    packet.pack(controller_id=event.joy,
                                event=JOYAXISMOTION,
                                event_dimension=event.axis,
                                event_value=event.value)
                    print(event.joy, JOYAXISMOTION, event.axis, event.value)
                    print("Payload Size:", sys.getsizeof(packet.get_payload().encode(format)))
                    print("----")
                elif event.type == JOYBALLMOTION:
                    self.joy[event.joy].ball[event.ball] = event.rel
                elif event.type == JOYHATMOTION:
                    self.joy[event.joy].hat[event.hat] = event.value
                elif event.type == JOYBUTTONUP:
                    self.joy[event.joy].button[event.button] = 0
                elif event.type == JOYBUTTONDOWN:
                    self.joy[event.joy].button[event.button] = 1
                    print(JOYBUTTONDOWN, event.joy, event.button)
                    packet.pack(controller_id=event.joy,
                                event=JOYBUTTONDOWN,
                                event_dimension=event.button,
                                event_value=1)
                    
                if remoteControl and len(packet.get_payload()) > 0:
                    self.clientSocket.send(packet.get_payload().encode(format))
                    #print(self.clientSocket.recv(1024).decode(format))

    def quit(self, status=0):
        pygame.quit()
        self.clientSocket.close()
        sys.exit(status)


if __name__ == "__main__":
    program = input_test()
    program.init()
    program.run()  # This function should never return
