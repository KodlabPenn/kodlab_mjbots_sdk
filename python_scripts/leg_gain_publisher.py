import lcm
from lcm_types.leg_gain import leg_gain

msg = leg_gain()

# current gains
msg.k = 3000
msg.b = 10
msg.kp = 60
msg.kd = 0.2
msg.kv = 0

lc = lcm.LCM()
lc.publish("leg_gains", msg.encode())