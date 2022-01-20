import lcm
from lcm_types.TVHGains import TVHGains

msg = TVHGains()

# current gains
msg.kv = 0
msg.tail_kp = 2
msg.tail_ki = 0
msg.tail_kd = 0.2
msg.tail_stance_ffwd = 0

lc = lcm.LCM()
lc.publish("jerboa_gains", msg.encode())