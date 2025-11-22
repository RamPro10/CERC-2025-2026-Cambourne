from hub import light_matrix
from motor import ERROR, run, stop, velocity
import motor
import runloop
import motor_pair
from hub import port
from hub import motion_sensor
import time

cm_per_rotation = 8.734
async def move_motor(motor_port, deg, velocity= 100):
    if motor_port == 'A':
        await motor.run_for_degrees(port.A, deg, velocity)
    elif motor_port == 'B':
        await motor.run_for_degrees(port.B, deg, velocity)
    elif motor_port == 'C':
        await motor.run_for_degrees(port.C, deg, velocity)
    elif motor_port == 'D':
        await motor.run_for_degrees(port.D, deg, velocity)
    elif motor_port == 'E':
        await motor.run_for_degrees(port.E, deg, velocity)
    elif motor_port == 'F':
        await motor.run_for_degrees(port.F, deg, velocity)

async def move_vertical_arm_up(deg, velocity= 100):
    await motor.run_for_degrees(port.B, deg, velocity)

async def move_vertical_arm_down(deg, velocity = 100):
    await motor.run_for_degrees(port.B, -deg, velocity)

async def rotate_horizontal_arm(deg, velocity= 100):
    await motor.run_for_degrees(port.C, deg, velocity)

async def move_right_wheel(distance, velocity=100):
    distance_in_degrees = int((distance / cm_per_rotation) * 360)
    await motor.run_for_degrees(port.D, -distance_in_degrees, velocity, stop=motor.HOLD)

async def move_left_wheel(distance, velocity=100):
    distance_in_degrees = int((distance / cm_per_rotation) * 360)
    await motor.run_for_degrees(port.F, distance_in_degrees, velocity, stop=motor.HOLD)

async def setup_motor():
    #motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    #await runloop.until(motion_sensor.stable)
    motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)

def pause(secs):
    time.sleep(secs)

def angle_value(angle):
    return angle % 3600


def move_direction(cur_angle, target_angle):
    half_circle = angle_value(cur_angle + 1800)
    if cur_angle<half_circle:
        if target_angle> cur_angle and target_angle<half_circle:
            return 1 # left
        else:
            return -1 # right
    else:
        if target_angle> half_circle and target_angle< cur_angle:
            return -1 # right
        else:
            return 1 #left





async def turn_to_angle(angle, velocity=50, direction=None, slowest_velocity=50, post_adjust=False, skip_gradient=False):

    target_angle = angle_value(int(10*angle))
    current_angle = angle_value(motion_sensor.tilt_angles()[0])

    if direction is None:
        steering = 100 * move_direction(current_angle, target_angle)
    elif direction == 1:
        steering = -100 # left
    else:
        steering = 100 # right

    init_error = angle_value(target_angle - current_angle)
    if init_error> 1800:
        init_error -= 3600
    if init_error==0:
        return

    m = (velocity - slowest_velocity)/abs(init_error)

    while True:
        current_angle = angle_value(motion_sensor.tilt_angles()[0])
        angle_error = angle_value(target_angle - current_angle)
        if(angle_error > 1800):
            angle_error = angle_error - 3600
        if skip_gradient:
            act_velocity = velocity
        else:
            act_velocity = int(m * abs(angle_error) + slowest_velocity)

        if abs(angle_error)<10:
            motor_pair.stop(motor_pair.PAIR_1)
            break
        motor_pair.move(motor_pair.PAIR_1, steering, velocity=act_velocity)

    if post_adjust and velocity>slowest_velocity:
        pause(0.1)

        current_angle = angle_value(motion_sensor.tilt_angles()[0])
        angle_error = angle_value(target_angle - current_angle)
        if abs(angle_error)>10 and abs(angle_error)<3590 and velocity>slowest_velocity:
            await turn_to_angle(angle, velocity=slowest_velocity, slowest_velocity=slowest_velocity, skip_gradient=skip_gradient)


async def turn_right(angle, velocity=50, slowest_velocity=50, skip_gradient=False):
    initial_angle = angle_value(motion_sensor.tilt_angles()[0])
    target_angle = angle_value(initial_angle - 10*angle)
    await turn_to_angle(int(target_angle/10), velocity=velocity, direction=-1, slowest_velocity=slowest_velocity, skip_gradient=skip_gradient)

async def turn_right_to_angle(angle, velocity=50, slowest_velocity=50, skip_gradient=False):
    await turn_to_angle(angle, velocity=velocity, direction=-1, slowest_velocity=slowest_velocity, skip_gradient=skip_gradient)

async def turn_left(angle, velocity=50, slowest_velocity=50, skip_gradient=False):
    initial_angle = angle_value(motion_sensor.tilt_angles()[0])
    target_angle = angle_value(initial_angle + 10*angle)
    await turn_to_angle(int(target_angle/10), velocity=velocity, direction=1, slowest_velocity=slowest_velocity, skip_gradient=skip_gradient)

async def turn_left_to_angle(angle, velocity=50, slowest_velocity=50, skip_gradient=False):
    await turn_to_angle(angle, velocity=velocity, direction=1, slowest_velocity=slowest_velocity, skip_gradient=skip_gradient)



async def move_forward(distance, velocity = 100, steering = 0, motor_break=True):
    distance_in_degrees = int((distance / cm_per_rotation) * 360)
    if motor_break:
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, distance_in_degrees, steering, velocity=velocity, stop=motor.HOLD)
    else:
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, distance_in_degrees, steering, velocity=velocity, stop=motor.CONTINUE)



async def move_backward(distance, velocity = 100, steering=0, motor_break=True):
    distance_in_degrees = int((distance / cm_per_rotation) * 360)
    if motor_break:
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, -distance_in_degrees, steering, velocity=velocity, stop=motor.HOLD)
    else:
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, -distance_in_degrees, steering, velocity=velocity, stop=motor.CONTINUE)



async def main():
    """
    #await move_motor('A', 360, 100) # not connected

    await move_motor('B', 360, 100) ## vertical motor
    await move_motor('C', 360, 100) ## top motor
    await move_motor('D', 360, 100) ## right wheel
    #await move_motor('E', 360, 100) ## ultra sonic
    await move_motor('F', 360, 100) ## left wheel

    """
    await setup_motor()
    #await move_forward(80, velocity=1000)
    runloop.run(
        move_vertical_arm_up(200, velocity=600),
        move_forward(30, velocity=1000)
    )
    await move_vertical_arm_down(250, velocity=600),
    await move_backward(10, velocity=600)

    runloop.run(
        move_vertical_arm_up(100, velocity=600),
        move_backward(30, velocity=1000, motor_break=False)
    )
    runloop.run(
        move_vertical_arm_down(100, velocity=600),
        move_backward(30, velocity=1000)
    )

    #runloop.run(
    #    move_vertical_arm_up(300, velocity=600),
    #    move_backward(20, velocity=100)
    #)
    #await move_vertical_arm_down(300, velocity=100)
    """
    runloop.run(
        move_backward(60, velocity=100),
        move_vertical_arm_up(200, velocity=50)
    )
    """
"""
    #await move_backward(10)
    #await move_arm_down(360)
    #await move_backward(20)

    runloop.run(
        move_forward(10),
        move_arm_down(720)
    )

    await move_backward(20)
"""

async def test_mission_radar():
    await setup_motor()

    """
    # picking up a sample
    await move_vertical_arm_up(500, velocity=300)
    await move_vertical_arm_down(420, velocity=300)
    await move_left_wheel(10, velocity=400)
    await move_vertical_arm_up(150, velocity=200)
    await move_left_wheel(-10, velocity=400)
    await move_forward(20, velocity=300)
    """
    await move_vertical_arm_up(500, velocity=300)
    await move_vertical_arm_down(500, velocity=900)
    """
    speed = 400
    await move_right_wheel(20, velocity=speed)
    await move_left_wheel(20, velocity=speed)
    await move_left_wheel(-20, velocity=speed)
    await move_right_wheel(-20, velocity=speed)
    #await move_vertical_arm_down(400)
    """

async def test_mission_shipping_lane():
    await setup_motor()
    #await move_vertical_arm_down(400, velocity=300)
    #await move_vertical_arm_up(150, velocity=300)
    #await move_vertical_arm_down(150, velocity=300)
    await rotate_horizontal_arm(-300, velocity=300)

async def test_mission_shipping_lane2():
    await setup_motor()
    #await move_vertical_arm_down(400, velocity=300)
    #await move_vertical_arm_up(150, velocity=300)
    #await move_vertical_arm_down(150, velocity=300)
    await rotate_horizontal_arm(300, velocity=300)

async def test_mission_shipping_lane3():
    await move_vertical_arm_up(200, velocity=300)

async def test_mission_shipping_lane4():
    await move_vertical_arm_down(200, velocity=300)

async def mission_shipping_lane():
    await setup_motor()
    move_speed = 800

    runloop.run(
        move_vertical_arm_up(150, velocity=300),
        move_forward(25, velocity=move_speed)
    )

    await move_right_wheel(9, velocity=move_speed)
    await move_forward(11, velocity=move_speed)
    await rotate_horizontal_arm(500, velocity=300)
    await move_vertical_arm_up(600, velocity=300)

async def test_movement():
    await setup_motor()
    await move_forward(30, velocity=800)



async def mission_01_shipping_lane_and_sonar():
    await setup_motor()

    if True:
        await move_forward(20, steering=-20, velocity=1000, motor_break=False)
        await move_forward(20, steering=0, velocity=1000, motor_break=False)
        await turn_to_angle(-110, velocity=400)
        await move_forward(21, steering=-5, velocity=1000, motor_break=True)


    if True:
        await move_vertical_arm_up(120,velocity=100)
        await turn_right(30, velocity=200)
        await move_vertical_arm_down(100,velocity=100)

    if True:
        await move_backward(5, steering=0, velocity=1000)
        await turn_to_angle(-85, velocity=400)

    if True:
        runloop.run(
            move_vertical_arm_up(300,velocity=200),
            move_forward(28, steering=2, velocity=1000)
        )
    if True:
        await rotate_horizontal_arm(500, velocity=300)
        await move_forward(2, steering=0, velocity=1000)
        await rotate_horizontal_arm(1300, velocity=300)

    if True:
        await rotate_horizontal_arm(-500, velocity=300)
        await move_backward(15, steering=0, velocity=1000)

    if True:
        await turn_to_angle(-60, velocity=400)
        await move_forward(30, velocity=1000, steering=0)
    if False: # get the sample
        await turn_to_angle(-130, velocity=400)
        await move_forward(15, steering=0, velocity=1000)
        await move_vertical_arm_down(350,velocity=200),
    if False: # return from the sample
        await move_backward(30, steering=-30, velocity=1000, motor_break=False)
        await move_backward(50, steering=5, velocity=1000)
    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)


async def mission_00_1_collection_with_shipping_lane():
    await setup_motor()
    mission_speed=600
    if True:
        await move_vertical_arm_up(500, velocity=400)
        await move_forward(25, steering=-30, velocity=mission_speed, motor_break=True)
        await move_forward(15, steering=-5, velocity=mission_speed, motor_break=True)
        await move_forward(38, steering=-10, velocity=mission_speed, motor_break=True)
        await turn_right(30, velocity=180)
        await move_vertical_arm_down(520, velocity=300)

        await turn_left(38, velocity=180, skip_gradient=True)
        await move_vertical_arm_up(200, velocity=400)
        await turn_to_angle(-110, velocity=180)

        await move_backward(30, steering=0, velocity=mission_speed),
        await turn_to_angle(-120, velocity=100)
        await move_vertical_arm_down(200, velocity=400)
        await move_forward(14, steering=-5, velocity=400)


        await move_vertical_arm_up(130,velocity=100)
        await turn_right(30, velocity=400, skip_gradient=True)
        await move_vertical_arm_down(130,velocity=200)
        await move_backward(20, steering=20, velocity=1000)

        await move_forward(50, steering=-15, velocity=1000)
    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)

async def mission_01_sonar_submirsible():
    await setup_motor()
    first_part=False
    second_part=False
    mission_speed = 800
    if True:
        await move_vertical_arm_up(300, velocity=300)
        await move_forward(32, steering=10, velocity=mission_speed, motor_break=False)
        await move_forward(45, steering=0, velocity=mission_speed, motor_break=True)

        await move_forward(4, steering=0, velocity=400)

        await rotate_horizontal_arm(-1350, velocity=300)

        await rotate_horizontal_arm(200, velocity=300)
        await move_backward(30, velocity=mission_speed, steering=0)
    if True:
        await move_forward(20, steering=20, velocity=mission_speed, motor_break=True)
    if True:
        runloop.run(
            move_forward(30, steering=3, velocity=mission_speed, motor_break=True),
            move_vertical_arm_down(250, velocity=300),
            rotate_horizontal_arm(200, velocity=300)
        )
        await move_forward(2, steering=-5, velocity=mission_speed, motor_break=True)
    if first_part:
        await move_vertical_arm_up(250, velocity=300)
        await runloop.sleep_ms(500)
        runloop.run(
            move_backward(15, velocity=1000, steering=-10),
            move_vertical_arm_down(300, velocity=300)
        )

    if second_part:
        await move_forward(27, steering=15, velocity=1000, motor_break=False)
        await move_forward(10, steering=-10, velocity=600, motor_break=False)
        await move_backward(10, velocity=1000)
        await turn_to_angle(80, velocity=100)
        await move_forward(20, steering=0, velocity=1000)
        await turn_to_angle(90, velocity=100)
        runloop.run(
                move_vertical_arm_up(300, velocity=300),
                move_forward(20, steering=0, velocity=1000)
            )
        await move_forward(20, steering=15, velocity=1000)
        await move_forward(8, steering=0, velocity=1000)
        await turn_left(32,velocity=300)

        await move_forward(50, velocity=1000, motor_break=False)
        await move_forward(15, steering=10, velocity=1000)
    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)




async def mission_02_shark_other():
    await setup_motor()
    if True:
        await move_forward(20, steering=-30, velocity=800, motor_break=False)
        await move_forward(57, steering=10, velocity=800, motor_break=False)
        await turn_to_angle(55, velocity=350)
        await move_forward(11, steering=0, velocity=800)

        await move_backward(8, steering=0, velocity=400)
        await turn_to_angle(70, velocity=350)
        await move_backward(6, steering=0, velocity=400)
        await move_backward(3, steering=-5, velocity=400)

        await turn_to_angle(95, velocity=350)
        await move_vertical_arm_down(530, velocity=200)
        await move_vertical_arm_up(10, velocity=200)
        await move_forward(11, velocity=300)
        await move_vertical_arm_up(100, velocity=200)
        await move_backward(8, steering=10, velocity=300)
        await turn_to_angle(82, velocity=350)
        await move_backward(5, steering=-5, velocity=300)
        await move_backward(12, steering=-7, velocity=300)
        await move_forward(7, velocity=300)
        await turn_left(20, velocity=350)
        await move_forward(5, steering=25, velocity=300)
        await move_forward(5, steering=0, velocity=300)
        await turn_to_angle(133, velocity=250)
        await move_backward(18, steering =-15, velocity=800)
        await turn_left(15,velocity=300, skip_gradient=True)
        await move_forward(15, velocity=1000)
        await turn_to_angle(160, velocity=350, skip_gradient=True)
        runloop.run(
            move_forward(40, steering=0, velocity=1000, motor_break=False),
            move_vertical_arm_up(400, velocity=200)
        )
        await move_forward(35, steering=-30, velocity=1000)
    #await runloop.sleep_ms(10000)
    #a wait move_vertical_arm_up(500, velocity=300)
    if False:
        await move_backward(7, steering=-30, velocity=400)

        await turn_to_angle(90, velocity=350)
        await move_vertical_arm_down(500, velocity=200)
        await move_vertical_arm_up(10, velocity=200)
        await move_forward(9, velocity=300)
        await move_vertical_arm_up(100, velocity=200)
        await move_backward(10, steering=-5, velocity=400)
        await move_backward(10, steering=2, velocity=300)
        await move_forward(5, velocity=300)
        await move_forward(10, steering =20, velocity=300)
    if False:
        await move_forward(10, steering=0, velocity=800)

        await move_backward(10, steering=0, velocity=800)
        await move_backward(8, steering=-15, velocity=600)
        await turn_to_angle(93, velocity=350)
        await move_vertical_arm_down(500, velocity=200)
        await move_forward(9, velocity=300)
        await move_vertical_arm_up(100, velocity=200)
        await move_backward(5, steering=0, velocity=300)
        await turn_to_angle(85, velocity=150)

        await move_backward(10, steering=-5, velocity=400)
        await move_backward(7, steering=-15, velocity=400)
        await move_forward(10, steering=0, velocity=400)

    if False:
        await move_backward(5, steering=0, velocity=1000)

        await move_backward(15, steering=-15, velocity=300)
        await turn_to_angle(90, velocity=350)
        await move_forward(15, velocity=300)
        await move_backward(10, steering=-25, velocity=600)
        await turn_to_angle(140, velocity=350)
        await move_backward(15, steering=-10, velocity=1000)
        await move_forward(60, velocity=1000, motor_break=False)
        await move_forward(15, steering=10, velocity=1000)
    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)

async def mission_04_scupa_other():
    await setup_motor()
    if True:
        await move_vertical_arm_up(350, velocity=200)
        await move_forward(20, steering=-30, velocity=1000, motor_break=False)
        await move_forward(60, steering=10, velocity=1000, motor_break=False)
        await move_forward(10, steering=0, velocity=400, motor_break=True)
        await turn_to_angle(90, velocity=300, slowest_velocity=100)

        await move_vertical_arm_down(200, velocity=200)

        await move_forward(8, steering=0, velocity=400, motor_break=True)
        await move_vertical_arm_up(100, velocity=200)

        await move_backward(30, steering=30, velocity=600)
        runloop.run(
            turn_to_angle(-5, velocity=350, slowest_velocity=100),
            move_vertical_arm_down(35, velocity=100)
        )
    await move_forward(7, steering=10, velocity=200)
    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)


async def mission_04_mast_scupa():
    await setup_motor()
    if True:
        await move_forward(60, steering=0, velocity=1000, motor_break=True)
        await turn_to_angle(-65, velocity=300)
        await move_vertical_arm_down(350, velocity=200)
        await move_backward(5, steering=10, velocity=300)
        await turn_to_angle(-83, velocity=300)
        await move_forward(15, steering=-3, velocity=200)
        await move_backward(15, steering=-25, velocity=600)
        await move_vertical_arm_up(350, velocity=200)
        await move_forward(25, steering=20, velocity=600)
        await turn_to_angle(90, velocity=350, slowest_velocity=150)
        await move_forward(2, velocity=300)
        await move_vertical_arm_down(195, velocity=60)
        await move_forward(6, velocity=300)
        await move_vertical_arm_up(100, velocity=200)
        await move_backward(20, steering=30, velocity=1000)
        await turn_to_angle(0, velocity=400, skip_gradient=True)
        runloop.run(
            move_backward(50, steering=10, velocity=1000, motor_break=False),
            move_vertical_arm_up(200, velocity=300)
        )
        await move_backward(30, steering=-15, velocity=1000)

    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)


async def mission_04_01_mast_scupa():
    await setup_motor()
    if True:
        await move_forward(60, steering=0, velocity=1000, motor_break=True)
        await turn_to_angle(-65, velocity=300)
        await move_vertical_arm_down(350, velocity=200)
        await move_backward(5, steering=10, velocity=300)
        await turn_to_angle(-83, velocity=300)
        await move_forward(15, steering=-3, velocity=200)
        await move_backward(15, steering=-25, velocity=600)
        await move_vertical_arm_up(350, velocity=200)
        await move_forward(25, steering=20, velocity=600)
        await turn_to_angle(90, velocity=350, slowest_velocity=150)
        await move_forward(2, velocity=300)
        await move_vertical_arm_down(195, velocity=60)
        await move_forward(6, velocity=300)
        await move_vertical_arm_up(100, velocity=200)
        await move_backward(20, steering=30, velocity=1000)
        await turn_to_angle(0, velocity=400, skip_gradient=True)
        runloop.run(
            move_backward(50, steering=10, velocity=1000, motor_break=False),
            move_vertical_arm_up(200, velocity=300)
        )
        await move_backward(30, steering=-15, velocity=1000)

    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)

async def mission_03_bring_boat_home():
    await setup_motor()
    if True:
        runloop.run(
            move_vertical_arm_up(200, velocity=600),
            move_forward(19, velocity=1000)
        )
    if True:
        await move_vertical_arm_down(250, velocity=600),
        await move_backward(15, velocity=600)

    if True:
        runloop.run(
            move_vertical_arm_down(400, velocity=600),
            move_backward(20, steering=10, velocity=1000, motor_break=False)
        )
    await move_backward(5, velocity=1000)

async def mission_04_return_boat_to_lock():
    await setup_motor()
    mission_speed = 700
    if True:
        await move_forward(10, steering=25, velocity=mission_speed, motor_break=False)
        await move_forward(15, velocity=mission_speed, motor_break=False)
        await move_forward(20, steering=-5, velocity=mission_speed, motor_break=False)
        await move_forward(10, steering=-5, velocity=mission_speed, motor_break=True)
        await turn_right(30, velocity=300, slowest_velocity=100)

        await move_forward(5, velocity=600, steering=-7, motor_break=False)
        await move_forward(10, velocity=600, steering=5, motor_break=False)
        await move_forward(10, velocity=600, steering=5, motor_break=False)
        await move_forward(6, velocity=600, motor_break=False)
        await move_forward(10, velocity=600, steering=3, motor_break=False)
        await move_forward(10, velocity=600, steering=15, motor_break=True)
        await move_forward(5, velocity=600, steering=15, motor_break=True)
        await move_forward(5, velocity=600, steering=-15, motor_break=True)
    if True:
        await move_backward(30, velocity=mission_speed, motor_break=False)
        await turn_to_angle(30, velocity=350, skip_gradient=True)
        await move_forward(30, velocity=mission_speed)

        await move_forward(15, steering=-15, velocity=mission_speed)
        await turn_to_angle(0, velocity=350, skip_gradient=True)
        await move_backward(50, velocity=1000, motor_break=False)
        await move_backward(40, steering=-10, velocity=1000, motor_break=False)
        await move_backward(30, steering=15, velocity=1000, motor_break=True)

    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)

async def mission_06_coral_nursury():
    await setup_motor()
    await move_forward(13, steering=2, velocity=400)
    await move_vertical_arm_up(100, velocity=200)
    runloop.run(
        move_vertical_arm_up(90, velocity=50),
        move_forward(14, steering=-4, velocity=600)
        )
    await runloop.sleep_ms(1500)
    await move_backward(10, velocity=1000)
    runloop.run(
        move_backward(20, velocity=1000),
        move_vertical_arm_down(190)
    )

async def mission_01_sonar_submirsible_org():
    await setup_motor()
    first_part=True
    if first_part:
        await move_forward(32, steering=10, velocity=1000, motor_break=False)
        runloop.run(
            move_forward(45, steering=0, velocity=1000, motor_break=True),
            move_vertical_arm_up(300, velocity=300)
        )
        await move_forward(4, steering=0, velocity=400)

    if first_part:
        await rotate_horizontal_arm(-1350, velocity=300)
    if first_part:
        await rotate_horizontal_arm(200, velocity=300)
        await move_backward(30, velocity=1000, steering=0)
    if first_part:
        await move_forward(20, steering=20, velocity=1000, motor_break=True)
    if first_part:
        runloop.run(
            move_forward(30, steering=3, velocity=1000, motor_break=True),
            move_vertical_arm_down(250, velocity=300),
            rotate_horizontal_arm(200, velocity=300)
        )
        await move_forward(2, steering=-5, velocity=1000, motor_break=True)
    if first_part:
        await move_vertical_arm_up(250, velocity=300)
        await runloop.sleep_ms(500)
        runloop.run(
            move_backward(15, velocity=1000, steering=-10),
            move_vertical_arm_down(300, velocity=300)
        )
    second_part=True
    if second_part:
        await move_forward(27, steering=15, velocity=1000, motor_break=False)
        await move_forward(10, steering=-10, velocity=600, motor_break=False)
        await move_backward(10, velocity=1000)
        await turn_to_angle(80, velocity=100)
        await move_forward(20, steering=0, velocity=1000)
        await turn_to_angle(90, velocity=100)
        runloop.run(
                move_vertical_arm_up(300, velocity=300),
                move_forward(20, steering=0, velocity=1000)
            )
        await move_forward(20, steering=15, velocity=1000)
        await move_forward(8, steering=0, velocity=1000)
        await turn_left(32,velocity=300)

    await move_forward(50, velocity=1000, motor_break=False)
    await move_forward(15, steering=10, velocity=1000)
    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)

async def mission_01_sonar_submirsible_org2():
    await setup_motor()
    first_part=True
    if first_part:
        await move_forward(32, steering=10, velocity=1000, motor_break=False)
        runloop.run(
            move_forward(45, steering=0, velocity=600, motor_break=True),
            move_vertical_arm_up(300, velocity=300)
        )
        await move_forward(4, steering=0, velocity=400)

    if first_part:
        await rotate_horizontal_arm(-1350, velocity=300)
    if first_part:
        await rotate_horizontal_arm(200, velocity=300)
        await move_backward(30, velocity=1000, steering=0)
    if first_part:
        await move_forward(20, steering=20, velocity=1000, motor_break=True)
    if first_part:
        runloop.run(
            move_forward(35, steering=3, velocity=1000, motor_break=True),
            move_vertical_arm_down(250, velocity=300),
            rotate_horizontal_arm(200, velocity=300)
        )
        await move_forward(2, steering=-5, velocity=1000, motor_break=True)
    if first_part:
        await move_vertical_arm_up(250, velocity=300)
        await runloop.sleep_ms(500)
        runloop.run(
            move_backward(15, velocity=1000, steering=-10),
            move_vertical_arm_down(300, velocity=300)
        )
    second_part=True
    if second_part:
        await move_forward(27, steering=15, velocity=1000, motor_break=False)
        await move_forward(10, steering=-10, velocity=600, motor_break=False)
        await move_backward(10, velocity=1000)
        await turn_to_angle(80, velocity=100)
        await move_forward(20, steering=0, velocity=1000)
        await turn_to_angle(90, velocity=100)
        runloop.run(
                move_vertical_arm_up(300, velocity=300),
                move_forward(20, steering=0, velocity=1000)
            )
        await move_forward(20, steering=15, velocity=1000)
        await move_forward(8, steering=0, velocity=1000)
        await turn_left(32,velocity=300)

    await move_forward(50, velocity=1000, motor_break=False)
    await move_forward(15, steering=10, velocity=1000)
    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)


import math

def sonar_error_distances():
    cur_angle = (int)(angle_value(motion_sensor.tilt_angles()[0]))/10
    error_angle=(cur_angle-18)
    rd = 50
    y = (16 + rd) * math.sin(math.radians(error_angle))
    back_error = round (y * math.tan(math.radians(42-error_angle)), 1)
    for_error = round(y / math.sin(math.radians(42-error_angle)), 1)
    return back_error, for_error

async def mission_01_sonar_submirsible_final():
    await setup_motor()
    if True:
        await move_vertical_arm_up(50, velocity=300)
        await move_forward(15, velocity=600)
        await turn_to_angle(23, velocity=250)

        runloop.run(
            move_forward(60, velocity=600),
            move_vertical_arm_up(250, velocity=300)
        )
        await move_forward(2, velocity=200)
        await rotate_horizontal_arm(-1250, velocity=300)
        await rotate_horizontal_arm(250, velocity=300)
        await move_backward(5, velocity=200)
        await turn_to_angle(12)
        await move_backward(7, velocity=600, steering=0)
        await turn_to_angle(60, velocity=300)

        runloop.run(
            move_forward(30, velocity=600),
            move_vertical_arm_down(300, velocity=300),
            rotate_horizontal_arm(80, velocity=300)
        )
        await turn_to_angle(55, velocity=300)

        await move_vertical_arm_up(180, velocity=200)
        await move_backward(2, velocity=50),
        await move_vertical_arm_up(180, velocity=200)
        await runloop.sleep_ms(200)
        await runloop.sleep_ms(300)
        runloop.run(
            move_backward(8, velocity=600, steering=0),
            move_vertical_arm_down(100, velocity=300)
        )
        await turn_to_angle(90, velocity=300)
        await move_forward(20, velocity=600)
        await move_forward(7, steering=-5, velocity=200)
        await move_forward(8, steering=-10, velocity=200)
        await move_forward(15, steering=-15, velocity=300)
        await turn_to_angle(85, velocity=200)
        await move_forward(10, velocity=600)
        await move_forward(15, steering=10, velocity=600)
        await move_forward(25, steering=20, velocity=600)
        await turn_left(15,velocity=300)
        await move_forward(50, velocity=1000, motor_break=False)
        await move_forward(15, steering=-20, velocity=1000)
    motor_pair.stop(pair=motor_pair.PAIR_1, stop=motor.HOLD)


async def mission_05_Habitat():
    await setup_motor()
    if True:
        runloop.run(
            move_forward(93, velocity=800),
            move_vertical_arm_up(300, velocity=800)
        )
        await turn_left(40, velocity=250, skip_gradient=True)
        await move_vertical_arm_down(300, velocity=800)
        await turn_right(50, velocity=350, skip_gradient=True)
        await move_vertical_arm_up(300, velocity=250)
        await move_backward(10, steering=0, velocity=300)
        await move_backward(9, steering=0, velocity=300)

        await turn_left(40, velocity=250, skip_gradient=True)
        runloop.run(
            move_vertical_arm_up(300, velocity=300),
            turn_to_angle(0, velocity=150)
            )
        await move_backward(40, velocity=1000, motor_break=False)
        await move_backward(50, steering=-15, velocity=1000, motor_break=True)


async def setup_motor():
    #motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    #await runloop.until(motion_sensor.stable)
    motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)

cm_per_rotation = math.pi*6.4

async def first_mission_2025():
    await setup_motor()
    await move_forward(10, velocity=500)
    await turn_right(90, velocity=300)
    await move_backward(10)



runloop.run(first_mission_2025())

#runloop.run(mission_00_1_collection_with_shipping_lane()) # working
#runloop.run(mission_01_sonar_submirsible_final()) #latest mission 01
#runloop.run(mission_02_shark_other()) # working
#runloop.run(mission_03_bring_boat_home()) # working
#runloop.run(mission_04_return_boat_to_lock()) #working
#runloop.run(mission_05_Habitat()) #working
#runloop.run(mission_06_coral_nursury()) # working

#runloop.run(mission_04_mast_scupa()) # working
########################################################################
########################################################################
########################################################################
