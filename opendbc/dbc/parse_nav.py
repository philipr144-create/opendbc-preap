#!/usr/bin/env python3
import cereal.messaging as messaging

def main():
    can_sock = messaging.sub_sock('can')
    print("Watching all MCU Nav & Map data... (Press CTRL+C to stop)", flush=True)

    last_968 = None
    last_760 = None

    while True:
        can_msgs = messaging.recv_sock(can_sock, wait=True)
        if can_msgs is None:
            continue

        for msg in can_msgs.can:
            # Message 968: Off-Ramps and Route Active
            if msg.address == 968 and len(msg.dat) >= 5:
                nav_active = (msg.dat[3] >> 5) & 1
                branch_dist = (msg.dat[4] & 0x1F) * 10
                left_ramp = (msg.dat[4] >> 6) & 1
                right_ramp = (msg.dat[4] >> 7) & 1

                state_968 = f"[968] Route Active: {nav_active} | Off-Ramp Dist: {branch_dist}m | L: {left_ramp} R: {right_ramp}"
                if state_968 != last_968:
                    print(state_968, flush=True)
                    last_968 = state_968

            # Message 760: Speed Limits (The one openpilot is using)
            elif msg.address == 760 and len(msg.dat) >= 7:
                mpp_speed = (msg.dat[6] & 0x1F) * 5
                
                state_760 = f"[760] MPP Speed Limit: {mpp_speed}"
                if state_760 != last_760:
                    print(state_760, flush=True)
                    last_760 = state_760

if __name__ == "__main__":
    main()
