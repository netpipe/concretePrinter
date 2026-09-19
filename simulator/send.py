import socket
 
s = socket.create_connection(("127.0.0.1", 9999))
 
cmds = [
    "G21",
    "G90",
    "G28",
    "G1 X100 Y0 Z40 F3000",
    "G1 X0 Y100 Z80 F3000",
    "G1 X-80 Y-40 Z20 F3000",
    "G28",
]
 
for c in cmds:
    s.sendall((c + "\n").encode())
    print(s.recv(4096).decode(errors="ignore"))
