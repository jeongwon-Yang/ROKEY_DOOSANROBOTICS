import rclpy
import time
import datetime
from enum import Enum
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30  # 작업 속도

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# 세그먼트 정의 - 디지털 시계의 각 세그먼트 (A, B, C, D, E, F, G)
class Segment(Enum):
    A = 0  # 상단 가로
    F = 1  # 좌측 상단 세로
    B = 2  # 우측 상단 세로
    G = 3  # 중앙 가로
    E = 4  # 좌측 하단 세로
    C = 5  # 우측 하단 세로
    D = 6  # 하단 가로

# 숫자 0-9의 세그먼트 패턴 (A-G 순서대로)
# 순서: A, F, B, G, E, C, D
DIGIT_PATTERNS = {
    0: [True, True, True, False, True, True, True],   # 0: ABCDEF (no G)
    1: [False, False, True, False, False, True, False],  # 1: BC
    2: [True, False, True, True, True, False, True],   # 2: ABDEG
    3: [True, False, True, True, False, True, True],   # 3: ABCDG
    4: [False, True, True, True, False, True, False],   # 4: BCFG
    5: [True, True, False, True, False, True, True],   # 5: ACDFG
    6: [True, True, False, True, True, True, True],   # 6: ACDEFG
    7: [True, False, True, False, False, True, False],  # 7: ABC
    8: [True, True, True, True, True, True, True],   # 8: ABCDEFG
    9: [True, True, True, True, False, True, True]    # 9: ABCDFG
}

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("digital_clock_full", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            wait,
        )
        from DR_common2 import posj
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

    # 각 세그먼트 위치의 관절 각도값 (J1, J2, J3, J4, J5, J6)
    # 4개 자리수 - 시간 십의 자리(0), 시간 일의 자리(1), 분 십의 자리(2), 분 일의 자리(3)
    SEGMENT_JOINT_POSITIONS = {
    # 첫 번째 자릿수 (시간 십의 자리)
    0: {
    Segment.A: [13.93, -4.65, 124.84, 1.28, 60.04, 90], # 상단 가로
    Segment.F: [8.36, -12.66, 131.45, -0.71, 63.10, 8.76], # 좌측 상단 세로
    Segment.B: [6.75, 1.49, 119.52, -2.74, 59.60, 8.76], # 우측 상단 세로
    Segment.G: [0.81, -5.49, 126.31, -5.82, 60.73, 90], # 중앙 가로
    Segment.E: [-6.28, -19.68, 137.05, -8.84, 57.72, 8.69], # 좌측 하단 세로
    Segment.C: [-5.02, -4.08, 126.05, -9.43, 52.00, 0.70], # 우측 하단 세로
    Segment.D: [-12.70, -7.72, 129.80, -12.73, 51.97, 90], # 하단 가로
    },
    # 두 번째 자릿수 (시간 일의 자리)
    1: {
    Segment.A: [9.97, 15.81, 102.35, 1.70, 61.79, 90], # 상단 가로
    Segment.F: [7.95, 11.28, 108.62, -4.27, 63.32, 4.46], # 좌측 상단 세로
    Segment.B: [6.14, 17.11, 100.50, -4.44, 57.22, 4.47], # 우측 상단 세로
    Segment.G: [2.64, 14.51, 105.05, -7.95, 59.92, 90], # 중앙 가로
    Segment.E: [-2.57, 7.73, 113.12, -9.76, 58.70, 4.45], # 좌측 하단 세로
    Segment.C: [-2.79, 17.34, 100.87, -7.41, 56.98, 4.45], # 우측 하단 세로
    Segment.D: [-13.454, 15.108, 108.57, -4.703, 58.268, 89.996] # 하단 가로
    },
    # 세 번째 자릿수 (분 십의 자리)
    2: {
    Segment.A: [10.227, 36.742, 70.582, -0.494, 72.751, 90], # 상단 가로
    Segment.F: [4.962, 29.072, 83.188, 0.024, 68.056, 4.594], # 좌측 상단 세로
    Segment.B: [4.364, 42.074, 61.222, 0.022, 76.9, 4.194], # 우측 상단 세로
    Segment.G: [-0.614, 35.375, 72.857, -0.638, 72.079, 90.0], # 중앙 가로
    Segment.E: [-5.477, 28.644, 84.301, 0.022, 67.699, -5.06], # 좌측 하단 세로
    Segment.C: [-5.195, 41.859, 61.456, 0.021, 77.143, -5.066], # 우측 하단 세로
    Segment.D: [9.859, 35.249, 72.87, -0.695, 72.081, 80.931], # 하단 가로
    },
    # 네 번째 자릿수 (분 일의 자리)
    3: {
    Segment.A: [8.111, 55.962, 35.973, -1.316, 79.854, 96.808], # 상단 가로
    Segment.F: [4.142, 54.26, 38.439, -0.639, 87.046, 4.201], # 좌측 상단 세로
    Segment.B: [3.656, 58.22, 32.373, -1.468, 73.012, 3.994], # 우측 상단 세로
    Segment.G: [1.316, 55.454, 36.961, -3.732, 78.999, 96.176], # 중앙 가로
    Segment.E: [-3.28, 49.61, 47.677, -3.906, 78.463, 3.995], # 좌측 하단 세로
    Segment.C: [-3.322, 56.751, 35.495, -2.955, 70.962, 3.994], # 우측 하단 세로
    Segment.D: [-6.928, 55.148, 38.067, -2.602, 77.757, 87.948], # 하단 가로
    }
    }

    # 블록 보관 위치 (한 곳으로 통일)
    # 주의: 이 값은 예시이며, 실제 로봇에서 측정한 값으로 교체해야 합니다.
    STORAGE_JOINT_POSITION = [20, -20, 110, 0, 70, 0]

    # 세그먼트 상태 추적 (초기에는 모든 세그먼트에 블록이 있다고 가정)
    segment_states = {
        0: {seg: True for seg in Segment},  # 시간 십의 자리
        1: {seg: True for seg in Segment},  # 시간 일의 자리
        2: {seg: True for seg in Segment},  # 분 십의 자리
        3: {seg: True for seg in Segment}   # 분 일의 자리
    }

    # 블록 픽업 및 제거 함수 (관절 위치 사용)
    def remove_block(joint_pos):
        # 관절 위치로 이동
        print(f"이동: 블록 위치 {joint_pos}")
        try:
            movej(posj(joint_pos), vel=VELOCITY, acc=ACC)
            
            # 그립 수행
            print("그립 수행")
            grip()
            
            # 보관 위치로 이동
            print(f"이동: 블록 보관 위치 {STORAGE_JOINT_POSITION}")
            movej(posj(STORAGE_JOINT_POSITION), vel=VELOCITY, acc=ACC)
            
            # 릴리즈 수행
            print("릴리즈 수행")
            release_gripper()
            
            return True
        except Exception as e:
            print(f"블록 제거 실패: {e}")
            return False

    # 현재 시간에 따라 필요한 세그먼트 패턴 계산 (시간:분)
    def get_time_segments():
        now = datetime.datetime.now()
        hour = now.hour
        minute = now.minute
        
        # 시간과 분을 각 자릿수로 분리
        h1 = hour // 10   # 시간 십의 자리
        h2 = hour % 10    # 시간 일의 자리
        m1 = minute // 10 # 분 십의 자리 
        m2 = minute % 10  # 분 일의 자리
        
        # 각 자릿수별 필요한 세그먼트 반환
        return {
            0: [DIGIT_PATTERNS[h1][seg.value] for seg in Segment],
            1: [DIGIT_PATTERNS[h2][seg.value] for seg in Segment],
            2: [DIGIT_PATTERNS[m1][seg.value] for seg in Segment],
            3: [DIGIT_PATTERNS[m2][seg.value] for seg in Segment]
        }

    # 특정 시간:분으로 테스트
    def test_specific_time(hour, minute):
        # 시간과 분을 각 자릿수로 분리
        h1 = hour // 10   # 시간 십의 자리
        h2 = hour % 10    # 시간 일의 자리
        m1 = minute // 10 # 분 십의 자리 
        m2 = minute % 10  # 분 일의 자리
        
        # 각 자릿수별 필요한 세그먼트 반환
        return {
            0: [DIGIT_PATTERNS[h1][seg.value] for seg in Segment],
            1: [DIGIT_PATTERNS[h2][seg.value] for seg in Segment],
            2: [DIGIT_PATTERNS[m1][seg.value] for seg in Segment],
            3: [DIGIT_PATTERNS[m2][seg.value] for seg in Segment]
        }

    # 디지털 시계 표시 업데이트 (블록 제거만 수행)
    def update_clock_display(target_segments=None):
        # 툴 및 TCP 설정
        set_tool("Tool Weight_2FG")
        set_tcp("2FG_TCP")
        
        # 초기 위치로 이동
        JReady = [0, 0, 90, 0, 90, 0]
        print("이동: 초기 위치로")
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        # target_segments가 제공되지 않으면 현재 시간 사용
        if target_segments is None:
            target_segments = get_time_segments()
            now = datetime.datetime.now()
            print(f"현재 시간: {now.hour:02d}:{now.minute:02d}")
        
        # 블록 제거만 수행 (필요 없는 세그먼트의 블록만 제거)
        for digit in range(4):  # 모든 4자리 (시:분)
            for seg in Segment:  # 각 세그먼트 순회
                current_state = segment_states[digit][seg]
                target_state = target_segments[digit][seg.value]
                
                if current_state and not target_state:
                    # 블록 제거 필요 (현재 있지만 필요 없음)
                    print(f"블록 제거: 자릿수 {digit}, 세그먼트 {seg.name}")
                    if remove_block(SEGMENT_JOINT_POSITIONS[digit][seg]):
                        segment_states[digit][seg] = False
        
        # 작업 완료 후 대기 위치로 이동
        print("시간 표시 업데이트 완료")
        movej(JReady, vel=VELOCITY, acc=ACC)

    # 테스트 메뉴 실행
    def run_test_menu():
        while True:
            print("\n==== 디지털 시계 테스트 메뉴 ====")
            print("1. 현재 시간으로 표시 업데이트")
            print("2. 특정 시간으로 테스트 (시:분)")
            print("3. 모든 세그먼트 초기화 (모든 블록 표시)")
            print("4. 종료")
            
            choice = input("선택하세요 (1-4): ")
            
            if choice == "1":
                print("현재 시간으로 업데이트 시작...")
                update_clock_display()
            
            elif choice == "2":
                try:
                    time_input = input("표시할 시간을 입력하세요 (HH:MM 형식, 예: 14:30): ")
                    hour, minute = map(int, time_input.split(':'))
                    
                    if 0 <= hour <= 23 and 0 <= minute <= 59:
                        print(f"{hour:02d}:{minute:02d}로 테스트 시작...")
                        target_segments = test_specific_time(hour, minute)
                        update_clock_display(target_segments)
                    else:
                        print("잘못된 시간입니다. 시간은 0-23, 분은 0-59 범위여야 합니다.")
                except (ValueError, IndexError):
                    print("잘못된 형식입니다. HH:MM 형식으로 입력하세요 (예: 14:30).")
            
            elif choice == "3":
                print("모든 세그먼트 초기화 (모든 블록 표시)...")
                # 모든 세그먼트가 필요하다고 설정 (8 표시)
                all_segments_on = {
                    0: [True for _ in Segment],
                    1: [True for _ in Segment],
                    2: [True for _ in Segment],
                    3: [True for _ in Segment]
                }
                # 이미 모든 블록이 있다고 가정하므로 상태만 업데이트
                for digit in range(4):
                    for seg in Segment:
                        segment_states[digit][seg] = True
                print("초기화 완료 (모든 블록이 표시됨)")
                
            elif choice == "4":
                print("테스트 종료")
                break
            
            else:
                print("잘못된 선택입니다. 다시 시도하세요.")

    # 메인 실행
    try:
        print("디지털 시계 제어 시작 (시:분 전체 표시)...")
        # 초기 상태: 모든 세그먼트에 블록이 있음
        for digit in range(4):
            for seg in Segment:
                segment_states[digit][seg] = True
        
        # 테스트 메뉴 실행
        run_test_menu()
        
    except KeyboardInterrupt:
        print("프로그램 종료 요청")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        print("정리 및 종료 중...")
        # 초기 위치로 복귀
        try:
            JReady = [0, 0, 90, 0, 90, 0]
            movej(JReady, vel=VELOCITY, acc=ACC)
        except Exception as e:
            print(f"초기 위치 복귀 실패: {e}")
        
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"rclpy 종료 실패: {e}")

if __name__ == "__main__":
    main()
