import curses
import socket
import sys

stdscr = curses.initscr()
stdscr.keypad(True)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

while True:
    c = stdscr.getch()
    if c == curses.KEY_UP:
       sock.sendto(bytes("f"), (sys.argv[1], 3333))
    elif c == curses.KEY_LEFT:
       sock.sendto(bytes("r"), (sys.argv[1], 3333))
    elif c == curses.KEY_RIGHT:
       sock.sendto(bytes("l"), (sys.argv[1], 3333))
    elif c == curses.KEY_DOWN:
       sock.sendto(bytes("b"), (sys.argv[1], 3333))
    elif c == ord('s'):
       sock.sendto(bytes("s"), (sys.argv[1], 3333))
    elif c == ord('q'):
       sock.sendto(bytes("s"), (sys.argv[1], 3333))
       break

curses.endwin()
