#!/usr/bin/env python3
import cereal.messaging as messaging

def sniff_safely():
    can_sock = messaging.sub_sock('can')
    print("Listening to CAN safely... (Press CTRL+C to stop)")
    while True:
        can_msgs = messaging.recv_sock(can_sock, wait=True)
        if can_msgs is not None:
            for msg in can_msgs.can:
                if msg.address == 69:
                    print(f"Buttons (Bus {msg.src}) | Raw Data: {msg.dat}")
                elif msg.address == 968:
                    print(f"Nav Data (Bus {msg.src}) | Raw Data: {msg.dat}")

if __name__ == "__main__":
    sniff_safely()
