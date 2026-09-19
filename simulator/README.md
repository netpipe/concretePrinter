# concretePrinter


Windows: com0com + a TCP/COM bridge
Linux: socat to create a virtual tty and forward to this TCP server




nc 127.0.0.1 9999

G21
G90
G28
G1 X100 Y0 Z40 F3000
G1 X0 Y100 Z80 F3000
G1 X-80 Y-40 Z20 F3000
G28