import lcm
from lcm_types.TVHGains import TVHGains

msg = TVHGains()

# current gains
msg.kv = 0.05
msg.tail_kp = 32
msg.tail_ki = 0
msg.tail_kd = 1.5
msg.tail_stance_ffwd = 1

lc = lcm.LCM()
lc.publish("jerboa_gains", msg.encode())