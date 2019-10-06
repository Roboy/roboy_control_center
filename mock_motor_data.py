import rospy
import roboy_middleware_msgs.msg
import yaml
import random

rospy.init_node("mock_motor_data")
motor_info = rospy.Publisher('roboy/middleware/MotorInfo', roboy_middleware_msgs.msg.MotorInfo, queue_size=1)
motor_state = rospy.Publisher('roboy/middleware/MotorState', roboy_middleware_msgs.msg.MotorState, queue_size=1)
config = None
with open("/home/letrend/workspace/roboy3/src/roboy_plexus/config/roboy3.yaml", 'r') as stream:
    try:
        config = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

print("initializing %d icebuses"%(config["number_of_icebuses"]))

number_of_icebuses = config["number_of_icebuses"]
number_of_motors_per_icebus = {}
for i in range(number_of_icebuses):
    name = "icebus_{}".format(i)
    number_of_motors_per_icebus[i] = config[name]["number_of_motors"]
    print(name)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    for i in range(number_of_icebuses):
        msg = roboy_middleware_msgs.msg.MotorInfo()
        msg.icebus = i
        control_mode = []
        for j in range(number_of_motors_per_icebus[i]):
            control_mode.append(0)
            msg.Kp.append(random.randint(0,10))
            msg.Ki.append(random.randint(0,10))
            msg.Kd.append(random.randint(0,10))
            msg.deadband.append(random.randint(0,10))
            msg.IntegralLimit.append(random.randint(0,10))
            msg.PWMLimit.append(random.randint(0,10))
            msg.pwm.append(random.randint(0,10))
            msg.communication_quality.append(random.randint(0,100))
            msg.gearBoxRatio.append(random.randint(0,100))
        msg.control_mode = control_mode
        motor_info.publish(msg)

        msg = roboy_middleware_msgs.msg.MotorState()
        msg.icebus = i
        for j in range(number_of_motors_per_icebus[i]):
            msg.setpoint.append(random.randint(-10000,10000))
            msg.encoder0_pos.append(random.randint(-10000,10000))
            msg.encoder1_pos.append(random.randint(-10000,10000))
            msg.displacement.append(random.randint(-10000,10000))
            msg.current.append(random.randint(0,8000))
        motor_state.publish(msg)

    rate.sleep()