import lcm
from lcm_types.LegGains import LegGains

msg = LegGains()

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