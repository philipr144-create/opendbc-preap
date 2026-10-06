#!/usr/bin/env python3
import cereal.messaging as messaging

def main():
    can_sock = messaging.sub_sock('can')
    print("Listening for Turn Signals... (Press CTRL+C to stop)")

    last_stalk = None

    while True:
        can_msgs = messaging.recv_sock(can_sock, wait=True)
        if can_msgs is not None:
            for msg in can_msgs.can:
                # 69 is STW_ACTN_RQ (Steering Wheel Actions)
                if msg.address == 69 and len(msg.dat) >= 3:
                    # Turn signal is Byte 2, bits 0 and 1
                    turn_stalk = msg.dat[2] & 0x03
                    
                    state = "IDLE"
                    if turn_stalk == 1:
                        state = "LEFT"
                    elif turn_stalk == 2:
                        state = "RIGHT"
                        
                    if turn_stalk != last_stalk:
                        print(f"Turn Signal: {state} (Raw Value: {turn_stalk})")
                        last_stalk = turn_stalk

if __name__ == "__main__":
    main()
