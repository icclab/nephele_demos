from http.server import HTTPServer, BaseHTTPRequestHandler
from time import sleep
import threading

class MetricsHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/metrics":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            # Simulating a healthy Prometheus target
            self.wfile.write(b'up{prometheus="kc1/kube-prometheus-operator"} 1\n')

def run_server():
    server = HTTPServer(("0.0.0.0", 8000), MetricsHandler)
    print("Starting fake Prometheus metrics server...")
    server.serve_forever()

# Start the server in a separate thread
server_thread = threading.Thread(target=run_server)
server_thread.start()

# Let the server run for 30 seconds before stopping
sleep(30)

print("Stopping the server to trigger the alert...")
server_thread.join()  

