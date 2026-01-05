import socket
import ctypes
import selectors

SERVER_PORT = 12345

class MSG_t(ctypes.Structure):
  _fields_ = [("event", ctypes.c_bool),
              ("intf_rss_dbm", ctypes.c_double),
              ("target_intf_rss_dbm", ctypes.c_double),
              ]
  def __init__(self):
    self.event: ctypes.c_bool = False
    self.intf_rss_dbm: ctypes.c_double = 0
    self.target_intf_rss_dbm: ctypes.c_double = 0
  def __str__(self):
     return f"""MSG_t
event: {self.event}
intf_rss_dbm: {self.intf_rss_dbm}
target_intf_rss_dbm: {self.target_intf_rss_dbm}
"""
  
def start_server():
  server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server_socket.bind(('141.213.15.85', SERVER_PORT))
  server_socket.listen(1)
  print(f"Server is listening on port {SERVER_PORT}...")

  # Use selectors to listen for connections
  sel = selectors.DefaultSelector()
  sel.register(server_socket, selectors.EVENT_READ)
  
  while True:
    for key, _ in sel.select():
      selectObj = key.fileobj
      if selectObj == server_socket:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr} has been established.")
        sel.register(client_socket, selectors.EVENT_READ)
      else:
        client_socket = selectObj
        data = client_socket.recv(ctypes.sizeof(MSG_t))
        if data:
          msg = MSG_t.from_buffer_copy(data)
          print(f"Received data: {msg}")
        else:
          print(f"Closing connection to {client_socket.getpeername()}")
          sel.unregister(client_socket)
          client_socket.close()

start_server()