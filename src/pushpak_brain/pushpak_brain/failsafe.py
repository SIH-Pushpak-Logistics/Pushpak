import math


def isolation_state(msg_age_s, alive_ids, timeout_s):
    if msg_age_s is None or not math.isfinite(msg_age_s) or msg_age_s > timeout_s:
        return True, 'no /pushpak/peers_alive update'
    if len(alive_ids) == 0:
        return True, 'no peer alive'
    return False, ''
