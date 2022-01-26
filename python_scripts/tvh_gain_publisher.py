import lcm
from lcm_types.TVHGains import TVHGains

msg = TVHGains()

# current gains
msg.kv = 3.6
msg.tail_kp = 15
msg.tail_ki = 0
msg.tail_kd = 0.22
msg.tail_stance_ffwd = 1

lc = lcm.LCM()
lc.publish("jerboa_gains", msg.encode())