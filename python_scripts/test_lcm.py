import lcm
from lcm_types.motor_log import motor_log

log = lcm.EventLog('logs/lcmlog-2021-11-02.04', "r")


for event in log:
    if event.channel == "EXAMPLE":
        msg = motor_log.decode(event.data)

        print("Message:")
        print("   timestamp (ms)   = %s" % str(msg.timestamp))
        print("   margin (ms)      = %s" % str(msg.mean_margin))
        print("")