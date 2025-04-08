import rclpy
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# 블록 높이 분류 기준
THRESHOLD_LONG = 59.0  # measured_z >= 59.0 → 긴 블록
THRESHOLD_MED = 48.4   # measured_z >= 48.4 및 < 59.0 → 중간 블록

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pick_and_place_height_sort", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            get_current_posx,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 / DR_common2 : {e}")
        return

    # 헬퍼 함수: 디지털 입력 대기
    def wait_digital_input(sig_num):
        wait(1)

    # 그립 및 릴리즈 동작 정의
    def release_gripper():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

    def grip():
        # 먼저 릴리즈 후 그립 수행
        release_gripper()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    def move_z(z):
        """
         z축방향으로 z만큼 움직이는 함수
        """
        cur_pos = get_current_posx()[0]
        cur_pos[2] += z
        print("cur_pos: ", cur_pos)
        movel(cur_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    # 오른쪽 팔레트(소스) 좌표 - 제공된 좌표
    right_pallet = {
        1: posx([499.593, -54.01, 33.924, 163.438, -179.996, 163.831]),
        3: posx([499.681, -156.649, 32.801, 120.777, -179.66, 121.32]),
        7: posx([396.538, -52.859, 35.433, 149.583, 179.211, 150.015]),
        9: posx([397.046, -154.307, 34.966, 157.157, 179.266, 157.72])
    }
    
    # 나머지 위치 추정 (3x3 그리드에서 보간)
    right_pallet[2] = posx([499.637, -105.33, 33.36, 142.11, -179.83, 142.58])   # 1과 3 사이
    right_pallet[4] = posx([448.07, -53.44, 34.68, 156.51, 179.60, 156.92])      # 1과 7 사이
    right_pallet[6] = posx([448.36, -155.48, 33.88, 138.97, 179.80, 139.52])     # 3과 9 사이
    right_pallet[5] = posx([448.17, -105.33, 34.24, 138.98, 179.62, 139.35])     # 4와 6 사이
    right_pallet[8] = posx([396.79, -103.58, 35.20, 153.37, 179.24, 153.87])     # 7과 9 사이
    
    # 왼쪽 팔레트(타겟) 좌표 - 제공된 좌표
    left_pallet = {
        1: posx([500.932, 147.612, 32.346, 74.386, 179.862, 74.597]),
        3: posx([500.107, 44.649, 33.936, 103.418, 179.863, 103.782]),
        7: posx([398.245, 146.449, 33.232, 8.72, -179.851, 8.705]),
        9: posx([397.837, 45.697, 34.557, 98.656, 179.897, 98.753])
    }
    
    # 나머지 위치 추정 (3x3 그리드에서 보간)
    left_pallet[2] = posx([500.52, 96.13, 33.14, 88.90, 179.86, 89.19])         # 1과 3 사이
    left_pallet[4] = posx([449.59, 147.03, 32.79, 41.55, 179.86, 41.65])        # 1과 7 사이
    left_pallet[6] = posx([448.97, 45.17, 34.25, 101.04, 179.88, 101.27])       # 3과 9 사이
    left_pallet[5] = posx([449.21, 96.13, 33.40, 53.69, 179.87, 53.73])         # 4와 6 사이
    left_pallet[8] = posx([398.04, 96.07, 33.90, 53.69, 179.92, 53.73])         # 7과 9 사이
    
    # 높이별 배치할 왼쪽 팔레트 위치
    long_positions = [1, 2, 3]    # 긴 블록 위치
    medium_positions = [4, 5, 6]  # 중간 블록 위치
    short_positions = [7, 8, 9]   # 짧은 블록 위치
    
    # 사용 가능한 타겟 위치 트래킹
    available_long_pos = long_positions.copy()
    available_medium_pos = medium_positions.copy()
    available_short_pos = short_positions.copy()

    # 툴 및 TCP 설정
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # 초기 위치 설정
    JReady = [0, 0, 90, 0, 90, 0]  # 홈 포지션

    # --- 메인 작업 시작 ---
    print("로봇 이동: 초기 위치")
    movej(JReady, vel=VELOCITY, acc=ACC)
    
    # 모든 블록 위치를 순회하며 소스(오른쪽)에서 타겟(왼쪽)으로 이동
    for pos in range(1, 10):
        print(f"\n========== 블록 {pos} 작업 시작 ==========")
        source_pos = right_pallet[pos]
        
        # 1. 소스 팔레트 안전 위치로 이동 (블록 위 60mm)
        safe_height = 60
        safe_pos = posx([source_pos[0], source_pos[1], source_pos[2] + safe_height, 
                         source_pos[3], source_pos[4], source_pos[5]])
        print(f"이동: 블록 {pos} 상단 안전 위치")
        movel(safe_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 2. 그리퍼 준비 (그리퍼 모으기)
        print("동작: 그리퍼 준비")
        grip()
        
        # 3. Force compliance 적용하여 블록 높이 측정
        # 3-1. Force compliance 모드 활성화
        print("Force compliance: 하강하여 접촉 감지 시작")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0],
                         dir=[0, 0, 1, 0, 0, 0],
                         mod=DR_FC_MOD_REL)
        
        # 3-2. 하강할 목표 위치 (예상 블록보다 낮게)
        #down_pos = posx([source_pos[0], source_pos[1], source_pos[2] - 20, 
        #                source_pos[3], source_pos[4], source_pos[5]])
        #movel(down_pos, vel=30, acc=30, ref=DR_BASE)  # 천천히 하강
        
        # 3-3. 힘 조건 확인 (접촉 감지)
        while not check_force_condition(DR_AXIS_Z, max=8):
            pass
        
        # 3-4. 접촉 감지 후 현재 위치 가져오기
        contact_pos = get_current_posx(ref=DR_BASE)[0]
        measured_z = contact_pos[2]
        release_compliance_ctrl()
        wait(1)
        move_z(10)
        print(f"접촉 감지됨: Z높이 = {measured_z:.2f}mm")
        
        # 4. 블록 높이 분류
        if measured_z >= THRESHOLD_LONG:
            block_type = "long"
        elif measured_z >= THRESHOLD_MED:
            block_type = "medium"
        else:
            block_type = "short"
        
        print(f"블록 분류: {block_type}")
        
        # 5. 그립을 위한 위치 계산
        # 5-1. 접촉 후 약간 상승 (그리퍼 위치 조정)
        grip_adjust = 8  # 그립 위치 조정값 (mm)
        grip_up_pos = posx([contact_pos[0], contact_pos[1], measured_z + grip_adjust, 
                           contact_pos[3], contact_pos[4], contact_pos[5]])
        print("이동: 그립 위치 조정을 위해 상승")
        
        release_gripper()  # 그리퍼 열기
        # movel(grip_up_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 5-2. 그립을 위해 적절한 위치로 하강
        grip_down_adjust = 0.5  # 그립 위치 미세 조정 (mm)
        grip_down_pos = posx([contact_pos[0], contact_pos[1], measured_z + grip_down_adjust, 
                             contact_pos[3], contact_pos[4], contact_pos[5]])
        print("이동: 그립 최적 위치로 하강")
        # movel(grip_down_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        move_z(-30)
        
        # 6. 그립 실행
        print("동작: 그립 수행")
        grip()
        
        # 7. 소스 팔레트 안전 위치로 복귀
        print("이동: 소스 팔레트 안전 위치로 복귀")
        movel(safe_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 8. 블록 들어올리기 (높이 더 올림)
        lift_height = 100
        lift_pos = posx([source_pos[0], source_pos[1], source_pos[2] + lift_height, 
                         source_pos[3], source_pos[4], source_pos[5]])
        print("이동: 블록 들어올리기")
        movel(lift_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 9. 블록 타입에 따라 타겟 위치 결정
        if block_type == "long":
            if not available_long_pos:
                print("경고: 긴 블록을 놓을 위치가 없습니다!")
                target_pos_idx = 1  # 기본 위치
            else:
                target_pos_idx = available_long_pos.pop(0)
        elif block_type == "medium":
            if not available_medium_pos:
                print("경고: 중간 블록을 놓을 위치가 없습니다!")
                target_pos_idx = 4  # 기본 위치
            else:
                target_pos_idx = available_medium_pos.pop(0)
        else:  # short
            if not available_short_pos:
                print("경고: 짧은 블록을 놓을 위치가 없습니다!")
                target_pos_idx = 7  # 기본 위치
            else:
                target_pos_idx = available_short_pos.pop(0)
                
        target_pos = left_pallet[target_pos_idx]
        print(f"선택된 타겟 위치: {target_pos_idx} ({block_type} 블록)")
        
        # 10. 타겟 팔레트 안전 위치로 이동
        target_safe_height = 80
        target_safe_pos = posx([target_pos[0], target_pos[1], target_pos[2] + target_safe_height, 
                               target_pos[3], target_pos[4], target_pos[5]])
        print("이동: 타겟 위치 상단 안전 위치")
        movel(target_safe_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 11. 타겟 팔레트 접근 위치로 이동 - 블록 타입에 따라 높이 조정
        # 긴 블록: 기준 높이
        # 중간 블록: 5mm 더 내림
        # 짧은 블록: 10mm 더 내림
        if block_type == "long":
            target_approach_height = 15  # 기준 높이
        elif block_type == "medium":
            target_approach_height = 10  # 5mm 더 내림
        else:  # short
            target_approach_height = 5   # 10mm 더 내림
            
        print(f"블록 타입 ({block_type})에 맞게 높이 조정: {target_approach_height}mm")
        target_approach_pos = posx([target_pos[0], target_pos[1], target_pos[2] + target_approach_height, 
                                   target_pos[3], target_pos[4], target_pos[5]])
        print("이동: 타겟 위치 접근")
        movel(target_approach_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 12. 릴리즈 동작 (블록 놓기)
        print("동작: 블록 릴리즈")
        release_gripper()
        
        # 13. 타겟 안전 위치로 복귀
        print("이동: 타겟 안전 위치로 복귀")
        movel(target_safe_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        print(f"블록 {pos} 이동 완료 ({block_type} -> 위치 {target_pos_idx})")
    
    # 작업 완료 후 홈 포지션으로 복귀
    print("\n모든 블록 이동 완료, 홈 포지션으로 복귀")
    movej(JReady, vel=VELOCITY, acc=ACC)
    
    print("모든 작업 완료!")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
