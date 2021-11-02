import lcm
from python_types.bulk_data import bulk_data


log = lcm.EventLog('lcmlog-2021-10-30.01', "r")


for event in log:
    if event.channel == "EXAMPLE":
        msg = bulk_data.decode(event.data)

        print("Message:")
        print("   timestamp   = %s" % str(msg.timestamp))
        print("   position    = %s" % str(msg.position))
        print("")