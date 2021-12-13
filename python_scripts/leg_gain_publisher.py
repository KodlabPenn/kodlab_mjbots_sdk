import lcm
from lcm_types.leg_gain import leg_gain

msg = leg_gain()

# current gains
msg.k = 400
msg.k_stiff = 1000
msg.b = 5
msg.b_stiff = 20
msg.kp = 100
msg.kd = 10
msg.kv = 0

msg.kv = 0

lc = lcm.LCM()
lc.publish("leg_gains", msg.encode())