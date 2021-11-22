import lcm
from lcm_types.leg_gain import leg_gain

msg = leg_gain()

# current gains
msg.k = 800
msg.b = 15
msg.kp = 120
msg.kd = 0.2
msg.kv = 0

msg.kv = 0
lc = lcm.LCM()
lc.publish("leg_gains", msg.encode())