# Python 3 server example
from http.server import BaseHTTPRequestHandler, HTTPServer
import time
import struct
import json

hostName = "192.168.1.68"
serverPort = 8000

class MyHandler(BaseHTTPRequestHandler):
    # protocol_version = 'HTTP/1.1'
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def do_GET(self):
        self.send_response(200)
        try:
            with open("pidsetpoint.json", 'r') as infile:
                pidsetpoint = json.load(infile)
            p = pidsetpoint['p']
            i = pidsetpoint['i']
            d = pidsetpoint['d']
            setpoint = pidsetpoint['setpoint']
        except IOError:
            p = 1
            i = 0
            d = 0
            setpoint = 0

        print(self.path)

        if self.path == "/":
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(bytes('<html><head><title> </title></head>', "utf-8"))
            self.wfile.write(bytes(f'<p>p: {p}, i: {i}, d: {d}, setpoint: {setpoint}</p>', "utf-8"))
            self.wfile.write(bytes('<form method="post">', "utf-8"))
            self.wfile.write(bytes(f'<input type="number" step="any" name="p" value="{p}">', "utf-8"))
            self.wfile.write(bytes(f'<input type="number" step="any" name="i" value="{i}">', "utf-8"))
            self.wfile.write(bytes(f'<input type="number" step="any" name="d" value="{d}">', "utf-8"))
            self.wfile.write(bytes(f'<input type="number" step="any" name="setpoint" value="{setpoint}">', "utf-8"))
            self.wfile.write(bytes('<input type="submit" value="Submit">', "utf-8"))
            self.wfile.write(bytes('</form>', "utf-8"))
        if self.path == "/pidsetpoint":
            data = bytearray()
            data += struct.pack("f", p)
            data += struct.pack("f", i)
            data += struct.pack("f", d)
            data += struct.pack("f", setpoint)
            # self.send_header("Connection", "keep-alive")
            self.send_header("Content-Length", str(len(data)))
            self.send_header('Content-type', 'application/octet-stream')
            self.end_headers()
            self.wfile.write(data)
        elif self.path == "/controller":
            self.end_headers()

    @staticmethod
    def _parse_numerical_form(post_data):
        items = post_data.split('&')
        form_data = dict()
        for item in items:
            i = item.rfind('=')
            assert(i >= 0 and i < len(item))
            form_data[item[0:i]] = float(item[i+1:])
        return form_data

    def do_POST(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location','/')
            self.end_headers()

            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            form_data = MyHandler._parse_numerical_form(post_data)
            with open('pidsetpoint.json', 'w') as outfile:
                json.dump(form_data, outfile)

if __name__ == "__main__":        
    webServer = HTTPServer((hostName, serverPort), MyHandler)
    print("Server started http://%s:%s" % (hostName, serverPort))

    try:
        webServer.serve_forever()
    except KeyboardInterrupt:
        pass

    webServer.server_close()
    print("Server stopped.")