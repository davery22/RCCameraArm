#!/usr/bin/env python

import sys
import io
import os
import shutil
from subprocess import Popen, PIPE
from string import Template
from struct import Struct
from threading import Thread
from time import sleep, time
from http.server import HTTPServer, BaseHTTPRequestHandler
from wsgiref.simple_server import make_server
import socket
import serial
from numpy import int8
from numpy import uint8

import picamera
from ws4py.websocket import WebSocket
from ws4py.server.wsgirefserver import WSGIServer, WebSocketWSGIRequestHandler
from ws4py.server.wsgiutils import WebSocketWSGIApplication

###########################################
# CONFIGURATION
WIDTH = 640
HEIGHT = 480
FRAMERATE = 24
HTTP_PORT = 8082
WS_PORT = 8084
COLOR = u'#444'
BGCOLOR = u'#333'
JSMPEG_MAGIC = b'jsmp'
JSMPEG_HEADER = Struct('>4sHH')
###########################################


class StreamingHttpHandler(BaseHTTPRequestHandler):
    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
            return
        elif self.path == '/jsmpg.js':
            content_type = 'application/javascript'
            content = self.server.jsmpg_content
        elif self.path == '/index.html':
            content_type = 'text/html; charset=utf-8'
            tpl = Template(self.server.index_template)
            content = tpl.safe_substitute(dict(
                ADDRESS='%s:%d' % (self.request.getsockname()[0], WS_PORT),
                WIDTH=WIDTH, HEIGHT=HEIGHT, COLOR=COLOR, BGCOLOR=BGCOLOR))
        else:
            self.send_error(404, 'File not found')
            return
        content = content.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', len(content))
        self.send_header('Last-Modified', self.date_time_string(time()))
        self.end_headers()
        if self.command == 'GET':
            self.wfile.write(content)


class StreamingHttpServer(HTTPServer):
    def __init__(self):
        super(StreamingHttpServer, self).__init__(
                ('', HTTP_PORT), StreamingHttpHandler)
        with io.open('index.html', 'r') as f:
            self.index_template = f.read()
        with io.open('jsmpg.js', 'r') as f:
            self.jsmpg_content = f.read()


class StreamingWebSocket(WebSocket):
    def opened(self):
        self.send(JSMPEG_HEADER.pack(JSMPEG_MAGIC, WIDTH, HEIGHT), binary=True)


class BroadcastOutput(object):
    def __init__(self, camera):
        print('Spawning background conversion process')
        self.converter = Popen([
            'avconv',
            '-f', 'rawvideo',
            '-pix_fmt', 'yuv420p',
            '-s', '%dx%d' % camera.resolution,
            '-r', str(float(camera.framerate)),
            '-i', '-',
            '-f', 'mpeg1video',
            '-b', '800k',
            '-r', str(float(camera.framerate)),
            '-'],
            stdin=PIPE, stdout=PIPE, stderr=io.open(os.devnull, 'wb'),
            shell=False, close_fds=True)

    def write(self, b):
        self.converter.stdin.write(b)

    def flush(self):
        print('Waiting for background conversion process to exit')
        self.converter.stdin.close()
        self.converter.wait()


class BroadcastThread(Thread):
    def __init__(self, converter, websocket_server):
        super(BroadcastThread, self).__init__()
        self.converter = converter
        self.websocket_server = websocket_server

    def run(self):
        try:
            while True:
                buf = self.converter.stdout.read(512)
                if buf:
                    self.websocket_server.manager.broadcast(buf, binary=True)
                elif self.converter.poll() is not None:
                    break
        finally:
            self.converter.stdout.close()


def get_t(data):
    return data[1]

def get_b(data):
    return data[4]

def get_y(data):
    return (data[8] << 1) + data[9]# - 512

def get_z(data):
    return (data[12]<<1)+data[13]# - 512


def main():
    print('Initializing camera')
    with picamera.PiCamera() as camera:
        camera.resolution = (WIDTH, HEIGHT)
        camera.framerate = FRAMERATE
        camera.vflip = True
        camera.hflip = True
        sleep(1) # camera warm-up time
        print('Initializing websockets server on port %d' % WS_PORT)
        websocket_server = make_server(
            '', WS_PORT,
            server_class=WSGIServer,
            handler_class=WebSocketWSGIRequestHandler,
            app=WebSocketWSGIApplication(handler_cls=StreamingWebSocket))
        websocket_server.initialize_websockets_manager()
        websocket_thread = Thread(target=websocket_server.serve_forever)
        print('Initializing HTTP server on port %d' % HTTP_PORT)
        http_server = StreamingHttpServer()
        http_thread = Thread(target=http_server.serve_forever)
        print('Initializing broadcast thread')
        output = BroadcastOutput(camera)
        broadcast_thread = BroadcastThread(output.converter, websocket_server)
        print('Starting recording')
        camera.start_recording(output, 'yuv')
        VZOOM = 1.0
        HZOOM = 1.0
        X = 0.0
        Y = 0.0
        diff = HZOOM/20.0
        zoom = 0
        button = 48
        try:
            print('Starting websockets thread')
            websocket_thread.start()
            print('Starting HTTP server thread')
            http_thread.start()
            print('Starting broadcast thread')
            broadcast_thread.start()
            remote = socket.socket()
            remote.bind(('', 8089))
            remote.listen(5)
            c, addr = remote.accept()

            print('Opening serial output')
            #ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            print('Looking for remote connection...')
            print('Remote connected from', addr)

            pic_name = 1;
            while True:
                data = c.recv(1024)
                if len(data) < 14:
                    continue
                if data[0] == 116 and get_t(data) > zoom and HZOOM > 0.0:
                    X += diff/2.0
                    Y += diff/2.0
                    HZOOM -= diff
                    VZOOM -= diff
                    zoom = get_t(data)
                    #print(zoom)
                    #print(data)
                elif data[0] == 116 and get_t(data) < zoom and HZOOM < 1.0:
                    X -= diff/2.0
                    Y -= diff/2.0
                    HZOOM += diff
                    VZOOM += diff
                    zoom = get_t(data)
                    #print(zoom)
                    #print(data)
                
                if(data[6] == 121 and data[10] == 122):
                    #print(data[6:14])
                    #ser.write(data[6:14])
                    y = get_y(data)
                    z = get_z(data)
                    y = int8(round((y/32)/3))
                    z = int8(round((z/32)/3))
                    y = uint8(y-4)
                    z = uint8(z-4)
                    print(y)
                    print(z)
                    send_data = [0x7F, y, z, 0x8F]
                    #ser.write(send_data)

                if data[3] == 98 and button != get_b(data):
                    camera.capture('image%02d.jpg' % pic_name, use_video_port = True)
                    pic_name = pic_name + 1;
                    button = get_b(data)
                camera.zoom = (X, Y, HZOOM, VZOOM)
                diff = HZOOM/20.0
                #camera.wait_recording(5)
                #HZOOM = HZOOM - diff
                #VZOOM = VZOOM - diff
                #camera.zoom = (X, Y, HZOOM, VZOOM)
        except KeyboardInterrupt:
            pass
        finally:
            print('Stopping recording')
            camera.stop_recording()
            print('Closing remote socket')
            c.close()
            remote.close()
            print('Waiting for broadcast thread to finish')
            broadcast_thread.join()
            print('Shutting down HTTP server')
            http_server.shutdown()
            print('Shutting down websockets server')
            websocket_server.shutdown()
            print('Waiting for HTTP server thread to finish')
            http_thread.join()
            print('Waiting for websockets thread to finish')
            websocket_thread.join()


if __name__ == '__main__':
    main()
