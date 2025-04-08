# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import numpy as np
import math
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,    
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            get_current_posx,
            trans,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)
    def move_and_transform(current_pos, delta_list):
        """
        current_pos에 delta_list(길이 6 배열)를 적용한 새 위치를 계산해 movel 수행.
        movel 후 최종 위치를 반환한다.
        """
        # 만약 delta_list가 여러 개의 배열 합이라면 np.array로 처리 후 합산
        # 여기서는 단일 배열(길이6)이라고 가정
        # delta_list가 이미 numpy array라면 필요에 따라 tolist() 수행
        if not isinstance(delta_list, np.ndarray):
            delta_list = np.array(delta_list, dtype=float)
        new_pos = trans(current_pos, delta_list.tolist(), DR_BASE, DR_BASE)
        new_pos = new_pos.tolist()
        movel(new_pos, vel=VELOCITY, acc=ACC)
        return new_pos

    def add_deltas(*args):
        """
        예: add_deltas(delta_H_U, delta_h_U)
           -> delta_H_U + delta_h_U의 요소별 합
        """
        return sum([np.array(a, dtype=float) for a in args])
    JReady = [0, 0, 90, 0, 90, 0]
    #posx([619.53, 173.615, 210.131, 59.731, 92.697, 89.96])
    # pos_STRART = posx([595.017, -154.899, 3.4, 55.684, 113.988, 87.988])
    # pos_STRAR_B = posx([590.094, -160.08, 3.4, 55.674, 113.991, 87.967])
    pos_STRART_U = posx([589.995, -160.023, 299.6, 50.7, 119, 90])
    pos_grip=posx([613.405, 167.458, 234.736, 59.74, 92.637, 89])
    # posj([-16.562, 30.159, 105.53, 98.111, 78.14, -46.273])
    delta_H_U=[0,0,96,0,0,0]
    delta_H_D=[0,0,-96,0,0,0]
    delta_h_U=[0,0,7,0,0,0]
    delta_h_D=[0,0,-7,0,0,0]
    delta_W_R=[0,80,0,0,0,0]
    delta_W_L=[0,-80,0,0,0,0]
    delta_W_B=[-80,0,0,0,0,0]
    delta_W_F=[80,0,0,0,0,0]
    delta_cal=[6,6,0,0,0,0]
    delta_cal_1=[2,2,0,0,0,0]
    delta_cal_2=[-4,0,0,0,0,0]
    set_tool("Tool Weight_2FG")
    set_tcp("gripper")
    #컵 겹쳤을때 h=8  안겹칠떄h=95
    #반경=76
    while rclpy.ok():
        print("movej")
        wait(1)
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        movej(JReady, vel=VELOCITY, acc=ACC)
        movel(pos_STRART_U, vel=VELOCITY, acc=ACC)
        print(get_current_posx())
        pos_STRAR_B= move_and_transform(pos_STRART_U, add_deltas(3*np.array(delta_H_D), 1.3*np.array(delta_h_D)))
        print(get_current_posx())
        pos_STRART= move_and_transform(pos_STRAR_B, add_deltas(1*np.array(delta_cal)))
        print(get_current_posx())
        grip()
        pos_cup_1_U = move_and_transform(pos_STRART, add_deltas(5*np.array(delta_W_R), 1.0*np.array(delta_h_U),delta_W_F))
        print(get_current_posx())
        pos_cup_1_D = move_and_transform(pos_cup_1_U, add_deltas(1*np.array(delta_h_D)))
        print(get_current_posx())
        release()
        
        pos_cup_2 = move_and_transform(pos_cup_1_D, add_deltas(1*np.array(delta_h_U)))
        grip()
        pos_cup_2_U = move_and_transform(pos_cup_2, add_deltas(delta_H_U))
        pos_cup_2_M = move_and_transform(pos_cup_2_U, add_deltas(delta_W_L))
        pos_cup_2_D = move_and_transform(pos_cup_2_M, add_deltas(delta_H_D,delta_h_D))
        release()

        pos_cup_3 = move_and_transform(pos_cup_2_D, add_deltas(delta_h_U))
        grip()
        pos_cup_3_U = move_and_transform(pos_cup_3, add_deltas(delta_H_U))
        pos_cup_3_M = move_and_transform(pos_cup_3_U, add_deltas(0.5*np.array(delta_W_R),(math.sqrt(3) / 2)*np.array(delta_W_B)))
        pos_cup_3_D = move_and_transform(pos_cup_3_M, add_deltas(delta_H_D,1*np.array(delta_h_D)))
        release()

        pos_cup_4 = move_and_transform(pos_cup_3_D, add_deltas(delta_h_U))
        grip()
        pos_cup_4_U = move_and_transform(pos_cup_4, add_deltas(delta_H_U))
        pos_cup_4_M = move_and_transform(pos_cup_4_U, add_deltas(1.5*np.array(delta_W_L),(math.sqrt(3) / 2)*np.array(delta_W_F)))
        pos_cup_4_D = move_and_transform(pos_cup_4_M, add_deltas(delta_H_D,delta_h_D))
        release()

        pos_cup_5 = move_and_transform(pos_cup_4_D, add_deltas(1*np.array(delta_h_U)))
        grip()
        pos_cup_5_U = move_and_transform(pos_cup_5, add_deltas(delta_H_U))
        pos_cup_5_M = move_and_transform(pos_cup_5_U, add_deltas(0.5*np.array(delta_W_R),(math.sqrt(3) / 2)*np.array(delta_W_B)))
        pos_cup_5_D = move_and_transform(pos_cup_5_M, add_deltas(delta_H_D,delta_h_D))
        release()

        pos_cup_6 = move_and_transform(pos_cup_5_D, add_deltas(1*np.array(delta_h_U)))
        grip()
        pos_cup_6_U = move_and_transform(pos_cup_6, add_deltas(delta_H_U))
        pos_cup_6_M = move_and_transform(pos_cup_6_U, add_deltas(0.5*np.array(delta_W_R),(math.sqrt(3) / 2)*np.array(delta_W_B)))
        pos_cup_6_D = move_and_transform(pos_cup_6_M, add_deltas(delta_H_D,delta_h_D))
        release()

        pos_cup_7 = move_and_transform(pos_cup_6_D, add_deltas(1*np.array(delta_h_U)))
        grip()
        pos_cup_7_U = move_and_transform(pos_cup_7, add_deltas(delta_H_U))
        pos_cup_7_M = move_and_transform(pos_cup_7_U, add_deltas(0.5*np.array(delta_W_R),(1.5*math.sqrt(3) / 2)*np.array(delta_W_F)))
        pos_cup_7_D = move_and_transform(pos_cup_7_M, add_deltas(1.25*np.array(delta_h_D)))
        release()

        pos_cup_8 = move_and_transform(pos_cup_7_D, add_deltas(delta_h_U))
        grip()
        pos_cup_8_U = move_and_transform(pos_cup_8, add_deltas(delta_H_U))
        pos_cup_8_M = move_and_transform(pos_cup_8_U, add_deltas(1*np.array(delta_W_L)))
        pos_cup_8_D = move_and_transform(pos_cup_8_M, add_deltas(delta_H_D,delta_h_D))
        release()

        pos_cup_9 = move_and_transform(pos_cup_8_D, add_deltas(delta_h_U))
        grip()
        pos_cup_9_U = move_and_transform(pos_cup_9, add_deltas(delta_H_U))
        pos_cup_9_M = move_and_transform(pos_cup_9_U, add_deltas(0.5*np.array(delta_W_R),(math.sqrt(3) / 2)*np.array(delta_W_B)))
        pos_cup_9_D = move_and_transform(pos_cup_9_M, add_deltas(delta_H_D,1.125*np.array(delta_h_D)))
        release()

        pos_cup_10 = move_and_transform(pos_cup_9_D, add_deltas(delta_h_U))
        grip()
        pos_cup_10_U = move_and_transform(pos_cup_10, add_deltas(delta_H_U))
        pos_cup_10_M = move_and_transform(pos_cup_10_U, add_deltas((math.sqrt(3) / 2)*0.5*np.array(delta_W_F)))
        pos_cup_10_D = move_and_transform(pos_cup_10_M, add_deltas(1.125*np.array(delta_h_D)))
        release()

        pos_cup_11 = move_and_transform(pos_cup_10_D, add_deltas(1.5*np.array(delta_h_U)))
        pos_cup_11_T_pose=[ pos_grip[i] if 2<i else d for i, d in enumerate(pos_cup_11)]
        movel(pos_cup_11_T_pose, vel=VELOCITY, acc=ACC)
        pos_cup_11_M = move_and_transform(pos_cup_11_T_pose, add_deltas(delta_cal,1.25*np.array(delta_h_U)))
        grip()
        pos_cup_11_U = move_and_transform(pos_cup_11_M, add_deltas(2*np.array(delta_H_U)))
        pos_cup_11_T_cup=[ -90 if i==5 else d for i, d in enumerate(pos_cup_11_U)]
        movel(pos_cup_11_T_cup, vel=VELOCITY, acc=ACC)
        pos_cup_11_T_cup_M = move_and_transform(pos_cup_11_T_cup, add_deltas(1*np.array(delta_cal)))
        pos_cup_11_T_cup_D = move_and_transform(pos_cup_11_T_cup_M, add_deltas(3/10*np.array(delta_H_D),delta_cal_2))
        release()
        break
    rclpy.shutdown()
if __name__ == "__main__":
    main()
