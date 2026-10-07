#!/usr/bin/env python3
"""XLeRobot 0.4.0 (SO101 팔 2개 + 2바퀴 베이스 + 헤드 카메라)용 Joy-Con
원격조작. teleop_so101_joint_nudge.py에서 발전시킨 스크립트.

2026-10-06 버전 (xlerobot_JoyCon_1006.py). 이전 버전 xlerobot_JoyCon_1001.py와의
차이는 CHANGELOG.md (버전별 변경 기록) 참고.

현재 상태 (2026-10-06, step 3) -- 모드 두 개, L3로 토글
(Joy-Con 두 개 모두 필요):

  ARM 모드 (시작 모드): 각 Joy-Con이 자기 쪽 팔을 조종 (2026-10-01 배치):
      스틱 좌/우, 상/하     -> shoulder_pan, shoulder_lift (기울인 만큼 비례 속도)
      X/B (L: 위/아래)      -> elbow_flex nudge
      자이로               -> wrist_roll / wrist_flex (범위 = 캘리브레이션 끝까지)
      ZR (L: ZL) 누름      -> 그리퍼를 조금씩 (놓으면 멈춤). 새로 누를 때마다
                              방향이 열기 <-> 닫기로 바뀜 (공식 예제 방식, 첫 누름 = 열기)
      plus (L: minus)      -> 손목 원점 복귀 (wrist_flex/roll = 0, 시작 자세)
    비어 있음: Y/A, home (L: 왼쪽/오른쪽 화살표, capture, L), R3, SL/SR.
    바퀴는 0으로 유지.
  모드와 무관: 오른쪽 R을 누른 채 오른쪽 스틱 -> 헤드 카메라 (그동안 오른팔은
    스틱을 무시). 헤드는 Joy-Con 두 개 + XLeRobot일 때만.
  WHEEL 모드: 두 팔(과 그리퍼, 손목 적분기)은 모드 진입 순간의 자세로
    고정(FROZEN)된다.
      왼쪽 Joy-Con 기울기 -> 베이스: 앞/뒤로 기울이면 전진/후진, 좌/우로
                            기울이면 제자리 회전 (둘 다 = 곡선 주행).
                            진입 시 잡아둔 중립 자세 대비 가속도계의
                            중력 방향으로 측정 -- TiltDrive 참고.
      왼쪽 ZL             -> 브레이크 토글 (래치 방식: 기울기와 상관없이
                            계속 정지; 다시 누르면 해제되고 중립 자세를
                            다시 잡는다). ARM 모드에서 ZL은 왼쪽 그리퍼.
      오른쪽 X/B/Y/A      -> 헤드 카메라 위/아래/왼쪽/오른쪽 (nudge).
      오른쪽 plus         -> 헤드 카메라를 캘리브레이션 중심(0)으로 복귀.
  L3 (왼쪽 스틱 클릭) -> ARM <-> WHEEL 토글 (2026-10-01 저녁, 사용자 요청).
    시작은 ARM, 한 번 누르면 WHEEL, 다시 누르면 ARM. R3는 이제 아무것도
    안 한다 (예전: L3 = WHEEL, R3 = ARM 절대 지정). 한 번 눌렀는데 두 번
    세는 것을 막으려고 MODE_TOGGLE_MIN_INTERVAL_S 안의 재입력은 무시한다.
  워치독: 어떤 Joy-Con이든 JOYCON_WATCHDOG_S 동안 입력 리포트가 끊기면
    그 팔의 입력을 멈춘다 (블루투스 연결이 끊기면 마지막 리포트가 --
    버튼이 눌린 상태일 수도 있는 채로 -- 그대로 남아버리기 때문). WHEEL
    모드에서는 바퀴에 브레이크를 걸고, 살아있는 데이터로 ZL을 다시 누를
    때까지 브레이크 상태를 유지한다.
  LED: ARM = 첫 번째 플레이어 LED, WHEEL = 네 개 모두 켜짐, WHEEL+브레이크
    = 네 개 모두 깜빡임.
  --wheel-dry-run: 모든 게 동작하지만 x.vel/theta.vel은 항상 0으로
    전송된다 -- 보냈을 바퀴 명령은 [dbg WHEEL] 줄에 출력된다.
    첫 실행 때 베이스를 실제로 움직이기 전에 기울기 방향(아래의
    WHEEL_FORWARD_* / WHEEL_TURN_*)을 확인하는 용도.

여기부터 "2026-09-29, step 2"까지는 ARM 모드 레이아웃의 변경 이력이며,
그대로 보존한다. (2026-10-01: 아래 배치는 예전 것 -- 스틱은 이제 사용자
요청으로 pan/lift에 쓰고, elbow는 X/B (위/아래)로, 그리퍼는 누르는 동안만
움직이게 바뀌었다. 위의 요약이 현재 배치.)

관절 공간 직접 버튼 제어 -- 향후 팔 2개짜리 바퀴형 SO101 플랫폼용
프로토타입으로, 하드웨어 없이 작성됨 (2026-09 세션: 테스트할 SO101도
컨트롤러도 없었음). joyconrobotics/lerobot 소스만 보고 추론했으므로 --
여기 있는 모든 숫자는 검증된 기본값이 아니라, 실제 하드웨어가 돌아오면
보정해야 할 추측값으로 취급할 것.

대화에서 합의한 최종 버튼 배치 (Joy-Con 하나가 팔 하나 + 바퀴 하나를
담당; 바퀴 베이스가 아직 없으므로 이 스크립트는 팔+그리퍼+손목 절반만
연결한다):
  스틱               -> 현재는 의도적으로 할당 안 함 (2026-09 세션: 어제
                         teleop_so101_stick.py 테스트에서 스틱이 여전히
                         약간의 움직임을 만든다는 걸 사용자가 발견하고,
                         여기서는 아예 아무것도 연결하지 말라고 요청함 --
                         읽지도 말 것). 이 버전에는 jc.wheel_velocity가
                         없다; 바퀴 베이스가 실제로 생기면 이 속성을
                         되살리지 말고 스틱을 새로 연결할 것.
                         (2026-09-29: 여전히 안 읽음 -- 바퀴는 스틱이 아니라
                         기울기로 가게 됐다. 스틱 CLICK만 모드 전환용으로
                         사용.)
  Y/A (R Joy-Con) 또는
  left/right (L)     -> 모터 1 (shoulder_pan) nudge -/+
  B/X (R) 또는
  down/up (L)        -> 모터 2 (shoulder_lift) nudge -/+ (2026-09-04:
                         첫 실기 테스트 후 사용자 피드백에 따라 X=+/B=-에서
                         뒤집음 -- 버튼 쌍은 같고 방향만 반대)
  ZR/R (R Joy-Con) 또는
  ZL/L (L)           -> 모터 3 (elbow_flex) nudge -/+ (2026-09-04:
                         같은 피드백에 따라 R=+/ZR=-에서 뒤집음 -- 버튼 쌍은
                         같고 방향만 반대). 사용자 요청으로 SL/SR에서 여기로
                         옮김 (2026-09 세션) -- SL/SR은 이제 비어 있음/미사용.
  home (R) / capture
  (L)                -> 그리퍼 토글 (사용자 요청으로 ZR/ZL에서 여기로
                         옮김).
  plus (R) / minus
  (L)                -> wrist_flex/wrist_roll을 0으로 원점 복귀 (2026-09-05:
                         예전엔 home/plus 둘 다 그리퍼를 토글했는데 분리함 --
                         이제 home/capture는 그리퍼 전용, plus/minus는 손목
                         원점 복귀 전용).
                         참고: home/plus/capture/minus는 이 스크립트에서
                         내장 부수효과가 전혀 없다 (이 클래스의
                         common_update()는 처음부터 새로 작성됐고 베이스
                         클래스나 FixedAxesJoyconRobotics의 버전을 절대
                         호출하지 않으므로, 그쪽의 reset_joycon()/위치 리셋
                         동작이 여기서는 실행되지 않는다) --
                         teleop_so101_stick.py와 달리 안전하게 다른 용도로
                         쓸 수 있다.
  자이로 roll/pitch  -> wrist_roll / wrist_flex. 둘 다 더 이상
                         jc.orientation_rad에서 읽지 않는다 (2026-09-05:
                         roll도 옮김, 아래 참고) -- 이제 둘 다 원시(raw)
                         자이로 누설 적분기(leaky integrator)로,
                         jc.wrist_roll_deg (gx)와 jc.wrist_flex_deg (gy).
                         이유와 방법은 WRIST_ROLL_*/WRIST_FLEX_* 상수 참고.
                         diagnose_wrist_axes.py (같은 디렉터리)가 원래
                         flex 수정의 근거가 된 실제 자이로/가속도 수치를
                         만든 스크립트 -- 다시 진단해야 할 일이 생기면
                         (예: 쥐는 방식이 바뀐 후) 다시 돌려볼 것.
                         이력: 처음에 wrist_roll은 jc.orientation_rad의
                         roll (atan2(ay,-az), 자이로+가속도 상보 필터)을
                         그대로 쓰고 pitch만 교체했었다. roll은 단독으로
                         비트는 테스트에서 깔끔하게 추종했기 때문. 그런데
                         2026-09-05 하드웨어 테스트에서 flex를 대략 45도
                         넘게 위로 꺾으면 의도적으로 비틀지 않았는데도
                         wrist_roll이 같이 반응하는 걸 발견했다 -- 이건
                         버그라기보다 그 공식의 피할 수 없는 물리적
                         특성이다: flex 축에 대한 실제 회전은 az (로컬
                         좌표계 Z축 방향의 중력 성분)를 실제 roll 회전과
                         정확히 같은 만큼 바꾸므로, az에 의존하는 roll
                         공식은 flex가 az를 눈에 띄게 움직일 만큼 커지면
                         flex를 크로스토크로 받아들인다. flex는 이미
                         gy만 쓰고 있었으므로 (가속도계 항이 없어서 roll
                         쪽 신호가 섞여 들어올 여지가 없음), roll도 똑같이
                         하는 것이 해결책이었다: 순수 gx 적분 -- 수학적으로
                         az/ay와 완전히 독립이므로 flex 움직임 (ax/az만
                         건드리고 gx는 절대 건드리지 않음)이 영향을 줄 수
                         없다. 부수효과: 예전의 "+-180도를 넘으면 추종을
                         잃는다"는 한계 (jc.orientation_rad의 wrap되는
                         오일러각 roll의 특성)는 wrist_roll이 더 이상 그
                         추정기를 거치지 않으므로 이제 해당 없음 -- 대신
                         새로운 실패 양상은 원시 각속도 적분기가 갖는
                         자이로 바이어스 드리프트이며, 이는
                         WRIST_ROLL_RECENTER_TAU_SEC가 flex와 같은 방식으로
                         억제한다.

이 스크립트는 현재의 단일 SO101 팔로워를 대상으로 한다 (이제 IK가 전혀
필요 없음 -- motor1/2/3을 관절 공간에서 직접 구동).

2026-09-29: teleop_so101_joint_nudge.py의 독립 실행형 복사본. 예전에
teleop_so101_motion.py에서 import하던 헬퍼 네 개 (JOINT_NAMES,
wrapped_error, read_current_positions, move_to_zero_position)를 아래에
인라인으로 넣었으므로, 이 파일은 joyconrobotics + lerobot에만 의존한다 --
이 저장소의 다른 파일은 필요 없다.

2026-09-29, step 1 -- XLeRobot 0.4.0 (`xlerobot_2wheels`)에서 양팔 제어:
JointNudgeJoyconRobotics를 이제 Joy-Con마다 하나씩 생성하고, 각
인스턴스가 XLeRobot의 XLerobot2Wheels 로봇 클래스의 팔 하나를 구동한다
(왼쪽 Joy-Con -> left_arm_*, 오른쪽 Joy-Con -> right_arm_*). 팔별 제어
로직 (pan/lift/elbow/gripper용 버튼, wrist_flex/wrist_roll용 자이로 누설
적분기)은 단일 팔 스크립트와 동일하고, 바뀐 건 그 주변 배관뿐이다:
  - 관절 이름에 로봇의 팔 접두사("left_arm_"/"right_arm_")가 붙는다,
    ArmBinding 참고;
  - 제어 틱마다 get_observation()/send_action() 한 번으로 두 팔 (두 버스)을
    모두 처리한다. 팔마다 로봇 객체를 따로 두지 않는다;
  - XLerobot2WheelsConfig를 use_degrees=True로 생성한다 -- 기본값은
    RANGE_M100_100 (-100..100 정규화)이라, 그대로 두면 이 파일의 모든
    도(degree) 단위 상수가 조용히 망가진다 (lerobot 자체의
    SO101FollowerConfig는 기본이 use_degrees=True이고, 단일 팔 튜닝도
    그 설정으로 했다);
  - 캘리브레이션 프롬프트는 XLerobot2Wheels.connect()가 직접 처리한다
    (ENTER = 파일에서 복원, 'c' = 재캘리브레이션). 그래서 예전의
    "Recalibrate?" 프롬프트는 이 경로에서 없어졌다;
  - (step 2에서 대체됨) 바퀴와 헤드 모터는 구동하지 않았음.
  - 카메라: XLerobot2WheelsConfig의 기본 카메라 dict는 비어 있다 (업스트림
    에서 항목이 전부 주석 처리됨); 이 스크립트는 카메라를 건드리지 않는다.
    카메라가 설정돼 있으면 get_observation()이 매 틱 읽는다.
예전의 단일 SO101 팔로워 경로는 `--robot so101` 뒤에 남겨뒀다. 튜닝된
동작을 벤치 리그에서 여전히 확인할 수 있도록 (거기엔 wheel 모드 없음).

2026-09-29, step 2 -- WHEEL 모드 + 헤드 카메라 + 워치독 + LED. 맨 위에
요약한 대로. 구현 메모:
  - joyconrobotics는 common_update()를 자체 스레드에서 돌린다 (solve_loop,
    ~100Hz); Teleop.step()은 메인 스레드에서 돈다 (~50Hz). 둘은 평범한
    속성으로만 소통한다: Teleop은 jc.mode를 쓰고; Joy-Con 스레드는 누름
    카운터 (l3/zl/head_recenter; r3는 2026-10-01 저녁에 없앰)를 증가시키기만 하며, Teleop이 그 값을
    마지막으로 본 값과 비교한다. 그래서 누름이 유실되지 않고 락도 필요 없다.
  - 워치독은 joyconrobotics 자체의 register_update_hook()로, JoyconRobotics가
    여는 세 장치 핸들 (joycon, gyro, button) 모두에서 입력 리포트마다
    타임스탬프를 찍고, 셋 중 가장 오래된 것을 기준으로 삼는다.
  - 헤드 모터: head_motor_1 (id 7) = pan, head_motor_2 (id 8) = tilt.
    XLeRobot의 7_xlerobot_2wheels_teleop_joycon.py의 코드가 하는 방식을
    따름 (up -> head_motor_2 +, left -> head_motor_1 +; 그 파일의 주석은
    반대로 적혀 있고 틀렸다). 방향은 하드웨어에서 미검증 -- 필요하면
    HEAD_TILT_UP_SIGN / HEAD_PAN_LEFT_SIGN을 뒤집고, X/B가 tilt 대신 pan을
    하면 HEAD_TILT_MOTOR / HEAD_PAN_MOTOR를 서로 바꿀 것.
  - x.vel (m/s, + = 전진)과 theta.vel (deg/s, + = 좌회전 / 반시계)은
    XLerobot2Wheels.send_action() 자체의 베이스 키다; 이 값을 두 바퀴의
    Goal_Velocity로 변환한다 (차동 구동).

왼쪽 Joy-Con 자이로 축 부호: joyconrobotics의 PythonicJoyCon이 이미 기본
설정으로 왼쪽 Joy-Con의 Y/Z IMU 축을 반전시켜서 (invert_left_ime_yz) 두
유닛이 같은 좌표계로 보고하게 한다. 아래의 WRIST_GYRO_SIGN은 그 위에
얹는 좌/우별 (roll, flex) 부호 표 -- 왼쪽은 미튜닝 (하드웨어가 다르게
말할 때까지 +1,+1); 하드웨어로 테스트한 건 오른손 파지뿐이다.

NUDGE_DEG_PER_TICK은 미튜닝 추측값 -- 하드웨어가 돌아오면 [dbg] 출력을
보면서 조정할 것.

MOTOR_LIMIT_DEG (motor1/2/3별)는 더 이상 단일 임시값이 아니다 --
2026-09-04: 실기 테스트에서 motor1 (shoulder_pan)이 210도 이상 가야 하는데
약 180도에서 멈추는 게 확인됐고, 원인은 예전의 공용 MOTOR_LIMIT_DEG=80
클램프가 shoulder_pan의 실제 캘리브레이션 범위보다 좁았기 때문이었다.
lerobot SO101Follower의 정규화 (MotorNormMode.DEGREES: 0도 = 캘리브레이션
range_min/range_max의 중점, 양방향 대칭 -- motors_bus.py의
_normalize/_unnormalize 참고)에 따르면, 각 관절의 실제 안전 반범위는
`(range_max - range_min) / 2 * 360 / 4095` 도이며, homing_offset과는
무관하다. 이제 main()이 시작할 때 연결된 로봇 자신의 robot.calibration에서
이 값을 읽어 (이 캘리브레이션 파일 하나에서 하드코딩하지 않고) 관절별로
넘기며, JOINT_LIMIT_MARGIN_DEG만큼 빼서 nudge로 관절을 물리적 하드
스톱까지 밀어붙이지 못하게 한다.
(헤드 카메라의 pan/tilt 한계도 같은 방식으로 구한다.)

설치 (최상위 README의 lerobot/joycon-robotics 단계에 추가로): XLeRobot의
로봇 클래스가 `lerobot.robots.xlerobot_2wheels`로 import 가능해야 한다 --
먼저 XLeRobot을 pull하고 (2026-09-18 왼쪽 바퀴 수정이 필요함;
connect_xlerobot()이 확인한다), 패키지를 lerobot 안으로 복사:
    git -C XLeRobot pull
    cp -r XLeRobot/software/src/robots/xlerobot_2wheels \\
          lerobot/src/lerobot/robots/
(그 config가 `..config`, 즉 lerobot.robots.config를 import하므로 lerobot의
robots 패키지 안에 있어야 한다.) 그 __init__.py가 ZMQ host/client도
import하므로 pyzmq가 설치돼 있어야 한다:
    pip install "pyzmq>=26.2.1,<28.0.0"

2026-10-06: 같은 폴더의 xlerobot_JoyCon_1006.py로 새 파일 (1001 파일은 그대로 둠).
2026-10-01: 같은 폴더의 xlerobot_JoyCon_1001.py로 새 파일 (0930 파일은 그대로 둠).
2026-09-30: xlerobot_JoyCon_0930/xlerobot_JoyCon_0930.py로 이동 (예전엔
저장소 루트의 xlerobot_JoyCon_0929.py) 및 Windows 지원 추가:
  - 시리얼 포트 기본값이 OS별로 다르다 (Linux에선 /dev/ttyACM0/1); Windows
    에는 안전한 기본값이 없으므로 --port1/--port2 (또는 --port)를 COMx로
    반드시 지정해야 하고, 빠뜨리면 찾은 COM 포트 목록을 출력한다;
  - --check-joycons는 페어링된 Joy-Con을 joyconrobotics가 찾는 방식
    (hid.enumerate) 그대로 나열하고, 시리얼 번호가 joyconrobotics의 형식
    검사를 통과하는지 알려준다 -- Windows에서 달라질 수 있는 유일한 부분 --
    로봇은 건드리지 않는다.
  Windows에서 Joy-Con 쪽: 둘 다 블루투스 설정에서 페어링하면 된다
  (hid-nintendo / joycond 불필요); joyconrobotics가 hidapi로 통신한다.

사용법:
    python xlerobot_JoyCon_1006.py --check-joycons    # Joy-Con 페어링/시리얼 확인만
    python xlerobot_JoyCon_1006.py                      # XLeRobot, 양팔 (+ wheel 모드)
    python xlerobot_JoyCon_1006.py --wheel-dry-run      # 같음, 단 바퀴 명령은 출력만
    python xlerobot_JoyCon_1006.py --arms right         # XLeRobot, 오른팔만 (wheel 모드 없음)
    python xlerobot_JoyCon_1006.py --robot so101        # 예전 단일 SO101 벤치 리그
    python xlerobot_JoyCon_1006.py --robot-id <id> --calibrate   # 아직 캘리브레이션 파일이 없을 때
"""

import argparse
import math
import os
import sys
import time
import traceback

import joyconrobotics.device as _joycon_device
from joyconrobotics import JoyconRobotics


def mac_with_colons(serial):
    """Windows의 hidapi는 블루투스 Joy-Con의 시리얼을 콜론 없는 MAC
    ("a05a5fc68e84")으로 보고하고; Linux는 "a0:5a:5f:c6:8e:84"로 보고한다.
    joyconrobotics는 콜론 형식만 받아들인다 (JoyCon.__init__ /
    JoyconRobotics.__init__의 시리얼 검사). 시리얼은 다른 데엔 쓰이지 않으므로
    -- 장치는 vendor/product id로 연다 -- 콜론을 다시 넣어주면 Windows도
    Linux와 정확히 같은 경로를 탄다. 그 외 값은 그대로 반환."""
    if (
        isinstance(serial, str)
        and len(serial) == 12
        and all(c in "0123456789abcdefABCDEF" for c in serial)
    ):
        return ":".join(serial[i:i + 2] for i in range(0, 12, 2)).lower()
    return serial


def _device_ids_with_colon_macs(*args, _orig=_joycon_device.get_device_ids, **kwargs):
    return [(vid, pid, mac_with_colons(serial)) for vid, pid, serial in _orig(*args, **kwargs)]


# joyconrobotics.device.get_R_id/get_L_id는 호출 시점에 자기 모듈 전역에서
# get_device_ids를 찾으므로, 이렇게 바꿔치기하면 JoyconRobotics.__init__에도 적용된다.
_joycon_device.get_device_ids = _device_ids_with_colon_macs

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def wrapped_error(target, current):
    """current에서 target까지의 최단 부호 각거리 (도 단위).
    그냥 `target - current`로 하면 +-180 경계 근처에서 터진다 (예:
    target=+179, current=-179는 실제로 2도 오차지 358도가 아님). 그러면
    P 제어 루프가 관절을 먼 쪽으로 최고 속도로 돌려버린다."""
    return (target - current + 180) % 360 - 180


def read_current_positions(robot):
    obs = robot.get_observation()
    return {key.removesuffix(".pos"): value for key, value in obs.items() if key.endswith(".pos")}


def move_to_zero_position(robot, prefixes=("",), duration=3.0, kp=0.5, control_freq=50):
    """`prefixes`에 해당하는 모든 팔의 모든 관절을 `duration`초 동안 0을 향해
    P 제어로 구동한다. prefixes=("",)는 단일 SO101 팔로워; XLeRobot은
    ("left_arm_", "right_arm_")을 쓴다."""
    print("Moving to zero position...")
    zero_positions = {f"{prefix}{name}": 0.0 for prefix in prefixes for name in JOINT_NAMES}
    step_time = 1.0 / control_freq
    for _ in range(int(duration * control_freq)):
        current = read_current_positions(robot)
        action = {
            f"{joint}.pos": current[joint] + kp * wrapped_error(target, current[joint])
            for joint, target in zero_positions.items()
            if joint in current
        }
        if action:
            robot.send_action(action)
        time.sleep(step_time)
    print("Reached zero position.")

# joyconrobotics solve_loop 틱당 값 (common_update()는 ~10ms마다 실행:
# sleep(0.01) + 처리 시간). 그래서 누르고 있으면 대략 50 deg/s로 목표가 움직인다.
# 미튜닝 -- 하지만 하드웨어 조작감은 이 값으로 판단했다.
NUDGE_DEG_PER_TICK = 0.5
JOINT_LIMIT_MARGIN_DEG = 10.0  # nudge가 캘리브레이션된 하드 스톱까지 가지 않도록 여유
# 2026-09-05: 이 두 값을 서로 바꿨다. 원래는 하드웨어 없이 OPEN=0/CLOSED=60으로
# 작성했는데; 실기 테스트에서 기본 시작 상태 (gripper_state=1.0, 여기서
# CLOSED라고 붙은 값을 보냄)가 물리적으로 열린(OPEN) 상태로 나와서, 라벨이
# 이 로봇의 서보 방향과 반대였다. 토글 로직이 아니라 값을 바꿔서 라벨을
# 바로잡고, 사용자 요청대로 기본 시작 상태도 물리적으로 닫힌 상태가 되게 했다.
GRIPPER_OPEN_DEG = 60.0
GRIPPER_CLOSED_DEG = 0.0
# 2026-10-01 (사용자 요청): 그리퍼는 더 이상 토글이 아니다. home (L: capture)를
# 누르고 있는 동안만 지금 방향으로 조금씩 움직이고, 놓으면 그 자리에 선다.
# 방향은 R (L: L)로 닫기 <-> 열기를 바꾼다. 시작은 닫기 방향.
# 2026-10-01 오후 (사용자 요청): 방향 전환은 공식 XLeRobot 예제
# (7_xlerobot_2wheels_teleop_joycon.py, ZR/ZL)처럼 -- 같은 버튼을 새로 누를
# 때마다 방향이 바뀌고, 누르고 있는 동안 그 방향으로 움직인다. 버튼은 현행
# home (L: capture) 유지. R (L: L)은 이제 비어 있다. 속도/범위는 우리 값 유지
# (공식은 0.4도/프레임, 0..90도).
# 2026-10-01 저녁 (사용자 요청): 버튼도 공식처럼 ZR (L: ZL)로. home/capture는
# 비었다. 왼쪽 ZL은 WHEEL 모드에서는 여전히 브레이크 (그때 그리퍼는 고정).
GRIPPER_DEG_PER_TICK = 0.3  # common_update 틱 (~100Hz)당 -> ~30 deg/s, 끝-끝 60도에 ~2초. 미튜닝.
GRIPPER_START_OPENING = False  # 첫 누름 전의 방향. False면 첫 누름이 "열기" (닫힌 채로 시작하므로)

# 2026-10-01 (사용자 요청): ARM 모드에서 스틱으로 모터 1 (pan, 좌우)과 모터 2
# (lift, 상하)를 움직인다 -- 버튼 nudge 대신. 기울인 만큼 비례 속도, 끝까지
# 밀면 버튼 nudge와 같은 속도 (NUDGE_DEG_PER_TICK). 예전에 스틱을 빼달라고 한
# 건 "안 건드려도 조금씩 움직인다"는 이유였으므로 데드존을 넉넉히 잡는다.
# 스틱 원값 (0..4095)은 Joy-Con에 저장된 스틱 캘리브레이션 (사용자 > 공장)으로
# -1..1로 정규화한다; 못 읽으면 STICK_FALLBACK_CAL.
STICK_DEADZONE = 0.25  # 축별 |값|이 이 이하면 0 (축별이라 위로 밀 때 pan이 같이 새지 않음)
STICK_FULL = 0.9  # 이 이상이면 최고 속도; 그 사이는 선형
# 부호: 미검증. 버튼과 같은 방향이 되게 잡음 -- 오른쪽 = pan + (A / 오른쪽 화살표와
# 같음), 위 = lift - (X / 위 화살표와 같음). 반대로 가면 뒤집을 것.
STICK_PAN_SIGN = 1.0
STICK_LIFT_SIGN = -1.0
# 스틱 클릭 (L3/R3) 중과 뗀 직후 이 시간 동안은 스틱 기울기를 무시한다
# (스틱 클릭으로 모드를 바꾸는 순간 스틱이 눌리며 흔들리는 것 때문).
STICK_CLICK_HOLDOFF_S = 0.3
STICK_FALLBACK_CAL = {"center": (2048, 2048), "below": (1300, 1300), "above": (1300, 1300)}

# wrist_flex: jc.orientation_rad의 pitch에서 읽지 않는다 (아래 참고) --
# 대신 원시 자이로로 만든다. 2026-09-04 실기 진단
# (diagnose_wrist_axes.py 출력): 세로로 쥐면 중력이 거의 전부 Joy-Con의
# 로컬 가속도 X축에 실려서, joyconrobotics의 가속도계 기반 pitch 추정
# (atan2(ax, sqrt(ay^2+az^2)))이 자체 포화점 (~1.31 rad)에 고정된다 --
# 바로 이 파지 방식에서만 생기는 짐벌락 모양의 데드존. 실제로 손목을 꺾을
# 때 원시 자이로 gy에는 진짜 신호가 실린다는 걸 하드웨어로 확인했다
# (테스트 중 +-0.5-0.6 rad/s 흔들림) -- 신호를 묻어버린 건 가속도계 보정 항
# (라이브러리 상보 필터에서 매 프레임 45% 가중치)이었지, 축을 잘못 쓴 게
# 아니었다.
# wrist_roll은 처음엔 여기서 jc.orientation_rad의 roll을 그대로 썼다
# (이 파지에서 roll의 ay/az 쌍은 중력 정렬 특이점 근처가 아니고, 단독
# 비트는 테스트에서 깔끔하게 추종했으므로) -- 하지만 나중에 roll도 거기서
# 옮긴 이유는 아래 WRIST_ROLL_* 블록 참고 (az를 통한 flex->roll
# 크로스토크, 전체 이야기는 모듈 docstring의 이력 메모에 있음).
#
# 해결: 라이브러리의 포화된 pitch를 믿는 대신 gy를 직접 적분한다 (누설
# 적분기, 자이로 드리프트에 대한 1차 하이패스 같은 것).
# WRIST_FLEX_GAIN_DEG_PER_RAD, WRIST_FLEX_LIMIT_DEG,
# WRIST_FLEX_RECENTER_TAU_SEC는 모두 미튜닝 첫 추측값 -- 하드웨어에서
# 느낌을 보고 조정할 것.
#
# 2026-09-04, 두 번째 하드웨어 피드백 (여전히 미튜닝, 첫 시도보다 덜
# 보수적일 뿐): 두 손목 관절 모두 감도를 올림 (flex 40->55, main()에 있는
# roll의 인라인 스케일 50->65), flex 자체 클램프도 넓힘 (45->70도, "동작은
# 하는데 범위가 너무 좁다"는 피드백에 따라 -- 참고로 flex가 "어떤 각도를
# 넘으면 반응을 안 하던" 원인은 센서 한계가 아니라 이 클램프였다. 위에
# 적은 wrist_roll의 진짜 짐벌락 한계는 여기 숫자를 올린다고 해결되지
# 않는 것과 다르다). flex 방향도 뒤집음 (컨트롤러를 아래로 기울이면 손목이
# 위로 갔음) -- 아래의 "-gy" 참고.
#
# 2026-09-04, 세 번째 (아래 네 번째에서 대체됨): "위로 접으면 꽤 올라가는데
# 아래로 접으면 거의 안 움직인다"를 고치려고 게인을 위/아래 상수로 분리해
# 봤다 (아래는 95로 증가). 근본 원인은 게인이 아니라 아래의 2초 감쇠
# tau였다 -- 네 번째 참고. 이력으로 남겨둠; 분리는 없앴고 다시 게인 하나.
#
# 2026-09-04, 네 번째: 위/아래 비대칭의 진짜 원인은
# WRIST_FLEX_RECENTER_TAU_SEC가 너무 짧았던 것 (2초) -- 감쇠는 Joy-Con을
# 일정한 기울기로 들고 있는 동안에도 무조건 0으로 끌어당기므로 (정지 시
# gy~0이라 감쇠에 맞서는 게 없음) "저절로 되돌아간다"로 느껴졌고, 그걸
# 이해하기 전에는 "한쪽 방향이 더 약하다"로 느껴졌다 (조금 더 느리게
# 적분되는 방향이 같은 감쇠에 더 많이 잃었음). 해결: tau를 늘려서 평범하게
# ~2초 들고 있는 동안이 아니라 오래 가만히 있어야 원점으로 돌아가게 함.
# 그래도 장기적인 자이로 바이어스 드리프트를 완전히 없앨 수는 없다 (애초에
# 감쇠가 있는 이유가 그거다 -- 위의 원래 2026-09-04 메모 참고), 다만 평소
# 사용에서 그 트레이드오프가 훨씬 덜 눈에 띄게 될 뿐이다.
# 피드백상 위 방향 게인 (55)이 적당했으므로 -- 이제 양방향에 적용, 위/아래
# 분리 없음.
WRIST_FLEX_GYRO_DT = 0.01  # joyconrobotics 내부 업데이트 주기와 일치 (solve_loop가 0.01초 sleep)
WRIST_FLEX_GAIN_DEG_PER_RAD = 71.5  # 원래 55, 피드백 따라 +30%. 적분된 gy (rad) -> 출력 도. 미튜닝.
WRIST_FLEX_LIMIT_DEG = 70.0  # flex 출력의 대칭 클램프. 미튜닝.
WRIST_FLEX_RECENTER_TAU_SEC = 20.0  # 가만히 있으면 flex가 대략 이 초 동안 0으로
# 새어 돌아간다 -- 자이로 적분 드리프트가 폭주하지 않게 억제하면서도, 평범하게
# 몇 초 들고 있을 때 눈에 띄게 되돌아가지는 않게. 미튜닝.
WRIST_FLEX_DECAY_PER_TICK = math.exp(-WRIST_FLEX_GYRO_DT / WRIST_FLEX_RECENTER_TAU_SEC)

# wrist_roll: 2026-09-05, flex->roll 크로스토크를 없애려고 wrist_flex와 완전히
# 같은 순수 자이로 누설 적분기 패턴으로 옮겼다 -- 이유는 모듈 docstring의
# 이력 메모 참고. 예전 WRIST_ROLL_GAIN (85, jc.orientation_rad의 roll에
# 적용)은 그대로 이어지지 않는다: 이제 기반 신호가 시간에 대해 적분한 원시
# gx라서 라이브러리의 필터링/스케일된 roll 라디안과 단위 스케일이 다르다.
# 그러니 여기의 85는 보존된 값이 아니라 그냥 재시작점 -- 느낌 보고 다시
# 튜닝할 것.
WRIST_ROLL_GYRO_DT = 0.01  # joyconrobotics 내부 업데이트 주기와 일치 (solve_loop가 0.01초 sleep)
WRIST_ROLL_GAIN_DEG_PER_RAD = 110.5  # 원래 85, 피드백 따라 +30%. 적분된 gx (rad) -> 출력 도. 미튜닝.
WRIST_ROLL_LIMIT_DEG = 70.0  # roll 출력의 대칭 클램프. 미튜닝.
WRIST_ROLL_RECENTER_TAU_SEC = 20.0  # WRIST_FLEX_RECENTER_TAU_SEC와 같은 이유. 미튜닝.
WRIST_ROLL_DECAY_PER_TICK = math.exp(-WRIST_ROLL_GYRO_DT / WRIST_ROLL_RECENTER_TAU_SEC)

# 2026-10-01: "손목이 캘리브레이션 끝까지 안 가고 ~70%만 간다" (사용자). 원인은
# 위의 고정 클램프 +-70도: XLeRobot 캘리브레이션 범위가 flex +-101, roll +-167도라
# flex는 69%, roll은 42%만 쓰고 있었다. True면 다른 관절처럼 캘리브레이션
# 반범위 - JOINT_LIMIT_MARGIN_DEG로 클램프한다 (flex ~+-91, roll ~+-157).
# 롤백: False로 바꾸면 예전 +-70 (WRIST_*_LIMIT_DEG) 그대로.
WRIST_LIMIT_FROM_CALIBRATION = True

# 손목 적분기가 쓰는 원시 gx/gy에 곱하는 좌/우별 (roll_sign, flex_sign) 배수.
# "right" = 테스트한 파지 (2026-09-05 튜닝, +1/+1이면 그 동작을 정확히 유지).
# "left"는 미튜닝: joyconrobotics가 이미 왼쪽 유닛의 Y/Z 축을 오른쪽 유닛
# 좌표계로 미러링하므로, +1/+1이면 같은 손 동작이 양쪽 손목을 같은 방향으로
# 움직여야 한다 -- 실제 왼손 테스트 결과가 다르면 하나 또는 둘 다 뒤집을 것.
WRIST_GYRO_SIGN = {"right": (1.0, 1.0), "left": (1.0, 1.0)}

# ---------------------------------------------------------------------------
# Step 2 (2026-09-29): 모드, 바퀴, 헤드 카메라, 워치독, LED.
# 아래 숫자는 전부 하드웨어 없이 정한 첫 추측값 -- 미튜닝.
# ---------------------------------------------------------------------------
MODE_ARM = "arm"
MODE_WHEEL = "wheel"

# 워치독: 가장 최근 입력 리포트가 이보다 오래된 Joy-Con은 연결 끊김으로
# 취급한다 (리포트는 보통 ~15ms마다 들어옴).
JOYCON_WATCHDOG_S = 0.3

# 바퀴 기울기. 기울기는 WHEEL 모드 진입 시 (그리고 브레이크 해제 시) 잡아둔
# 중립 자세로부터 왼쪽 Joy-Con이 회전한 양이며, 가속도계의 중력 방향으로
# 측정한다 -- 손목에 쓰는 자이로 적분기와 달리 절대값이고 드리프트가 없다.
# 이 방식으로는 중력에 수직인 축에 대한 회전만 관측할 수 있다 (컨트롤러를
# 수직축 기준으로 비트는 건 보이지 않음). 그래서 회전이 비틀기가 아니라
# 옆으로 기울이기다. 결과는 Joy-Con 자체 좌표계의 회전 벡터 (인덱스 0/1/2 =
# x/y/z; 부호 규칙은 자이로와 같음: + = 그 축에 대한 오른손 법칙 회전).
# 손목 튜닝에 쓴 세로 파지 (중력이 로컬 X 방향)에서는, 앞/뒤 기울기가 로컬 Y
# 축 회전 (wrist_flex가 적분하는 것과 같은 축)이고 좌/우 기울기가 로컬 Z축
# 회전이다. 축은 고정된 바디 축이므로 중립 자세가 반드시 그 세로 파지여야
# 한다: 눕혀서 들면 (중력이 Z 방향) Z 채널이 전혀 움직일 수 없고 옆 기울기가
# X로 가는데, X는 읽지 않는다.
# 부호는 테스트 전엔 알 수 없다: --wheel-dry-run으로 한 번 돌려서 앞으로,
# 그리고 왼쪽으로 기울여 보고, 출력된 x.vel / theta.vel이 음수로 나오면
# WHEEL_FORWARD_SIGN / WHEEL_TURN_SIGN을 뒤집을 것.
# 2026-09-30 실측 (실제 왼쪽 Joy-Con, 사용자의 실제 파지, 60초 로그): 중립에서
# 중력이 로컬 X가 아니라 로컬 -Z 방향이었다 (g = (-0.20, +0.09, -0.97)).
# 앞/뒤 기울기 -> 로컬 Y 회전 (-35 / +57도, 위 가정과 같음 -- 그래서 전진/
# 후진은 됐다), 좌/우 기울기 -> 로컬 X 회전 (왼쪽 -60, 오른쪽 +46도; 이때
# Z는 +-12도뿐이라 데드존 10도를 겨우 넘어 "회전이 전혀 안 먹는" 것처럼
# 보였다). 그래서 회전 축을 X로 바꾸고, 왼쪽 기울기가 +theta(좌회전)가
# 되도록 부호를 -1로 한다.
WHEEL_FORWARD_AXIS = 1  # Joy-Con 로컬 Y
WHEEL_FORWARD_SIGN = -1.0  # 2026-09-30 실측+사용자 요청: 앞으로 기울임 = Y 음수 -> 전진(+)이 되도록 뒤집음
WHEEL_TURN_AXIS = 0  # Joy-Con 로컬 X (실측: 사용자 파지에서 좌/우 기울기)
WHEEL_TURN_SIGN = -1.0  # 실측: 왼쪽 기울기 = X 음수 -> 좌회전(+)이 되도록
WHEEL_TILT_DEADZONE_DEG = 10.0  # |기울기|가 이보다 작으면 -> 0 (손떨림, 대충 잡은 중립)
WHEEL_TILT_FULL_DEG = 30.0  # |기울기|가 이 이상이면 -> 최고 속도; 그 사이는 선형
WHEEL_TILT_CUTOFF_DEG = 75.0  # 전체 기울기가 이보다 크면 -> 0 (컨트롤러를 내려놓음 / 떨어뜨림) (2026-09-30: 60 -> 75, 실측 좌회전 기울기가 ~60도였음; 내려놓기는 ~90도)
WHEEL_MAX_LINEAR_MPS = 0.08  # 2026-10-06: 0.1 (XLerobot2Wheels 자체의 "slow" 속도 단계) x 0.8, 사용자 요청
WHEEL_MAX_ANGULAR_DEGPS = 24.0  # 2026-10-06: 30 x 0.8, 위와 같음
WHEEL_NEUTRAL_SETTLE_S = 0.3  # L3/ZL 후 이만큼 기다렸다가 중립을 잡음 (누를 때의 흔들림)
ACCEL_LPF_TAU_S = 0.1  # 가속도계 로우패스 (손 떨림 / 선형 가속도 제거)
ACCEL_VALID_MIN_G = 0.3  # 필터된 |가속도|가 이 범위 밖이면 = 쓸 만한 중력 값 없음
ACCEL_VALID_MAX_G = 3.0
assert 0.0 <= WHEEL_TILT_DEADZONE_DEG < WHEEL_TILT_FULL_DEG <= WHEEL_TILT_CUTOFF_DEG

# 헤드 카메라 (XLeRobot의 2모터 헤드에 달린 중앙 카메라, bus1 id 7/8).
# 역할은 XLeRobot의 7_xlerobot_2wheels_teleop_joycon.py의
# handle_joycon_input() 코드를 따른다 (그 인라인 주석은 1/2가 바뀌어 있음).
HEAD_TILT_MOTOR = "head_motor_2"
HEAD_PAN_MOTOR = "head_motor_1"
HEAD_TILT_UP_SIGN = 1.0  # 미검증: XLeRobot 예제: up -> head_motor_2 +
HEAD_PAN_LEFT_SIGN = 1.0  # 미검증: XLeRobot 예제: left -> head_motor_1 +
HEAD_NUDGE_DEG_PER_TICK = 1.26  # 메인 루프 틱 (~50Hz)당 -> 누르고 있으면 ~63 deg/s (2026-10-06: 0.4 -> 0.7 (사용자 요청 "1.5~2배") -> 같은 날 x1.8 = 1.26 (사용자 요청))
# 2026-10-01 (사용자 요청): 오른쪽 R을 누르고 있는 동안 오른쪽 스틱 -> 헤드 카메라
# (오른쪽 = pan 오른쪽, 위 = tilt 위; 같은 날 저녁 ZR -> R로 바꿈, ZR은 그리퍼).
# ARM/WHEEL 모드와 무관하게 동작한다. ARM
# 모드에서는 그동안 오른쪽 스틱이 팔을 움직이지 않고, R을 뗀 뒤 스틱이 한 번
# 데드존으로 돌아와야 다시 팔로 간다. 끝까지 밀면 HEAD_NUDGE_DEG_PER_TICK과 같은
# 속도 (스틱 데드존/램프는 STICK_*). WHEEL 모드의 X/B/Y/A 카메라 버튼도 그대로 있다.

# 플레이어 LED 패턴 (비트 i = LED i+1).
LED_ARM_PATTERN = 0b0001
LED_WHEEL_PATTERN = 0b1111  # WHEEL 모드: 켜짐; WHEEL + 브레이크: 깜빡임

# L3 토글: 이 시간 안에 다시 눌린 L3는 무시 (한 번 누름이 두 번 세어져 제자리로
# 돌아오는 것 방지). 의도적인 빠른 두 번 누름도 막히지만 모드 전환에는 문제 없음.
MODE_TOGGLE_MIN_INTERVAL_S = 0.4

PRESS_COUNTERS = ("l3_presses", "zl_presses", "head_recenter_presses", "wrist_reset_presses")


def joint_half_range_deg(calibration, joint, margin_deg=JOINT_LIMIT_MARGIN_DEG, resolution=4096):
    """관절이 캘리브레이션된 0점에서 움직일 수 있는 안전 +-도.
    lerobot의 DEGREES 정규화 기준 (0 = range_min/range_max의 중점)."""
    cal = calibration[joint]
    half_range_ticks = (cal.range_max - cal.range_min) / 2.0
    half_range_deg = half_range_ticks * 360.0 / (resolution - 1)
    return max(0.0, half_range_deg - margin_deg)


def gravity_unit(accel_g):
    """필터된 가속도계 벡터 (g 단위)를 정규화. 쓸 만한 값이 없으면 None
    (아직 수신 전이거나, 크기가 1 g에서 너무 멀 때)."""
    if accel_g is None:
        return None
    x, y, z = accel_g
    norm = math.sqrt(x * x + y * y + z * z)
    if not (ACCEL_VALID_MIN_G <= norm <= ACCEL_VALID_MAX_G):
        return None
    return (x / norm, y / norm, z / norm)


def tilt_rotation_deg(neutral, current):
    """컨트롤러가 `neutral`에서 `current`로 회전한 양 (둘 다 컨트롤러 자체
    좌표계에서의 중력 단위벡터)을 (도 단위 회전 벡터, 도 단위 전체 각도)로
    반환. 컨트롤러가 바디 축 k에 대해 +a만큼 돌면, 컨트롤러에서 본 중력은
    -a만큼 돌고, current x neutral = sin(a) * k (중력에 수직인 성분) --
    따라서 이 함수는 +a*k를 반환하며, 자이로와 같은 부호 규칙이다."""
    nx, ny, nz = neutral
    cx, cy, cz = current
    rx = cy * nz - cz * ny
    ry = cz * nx - cx * nz
    rz = cx * ny - cy * nx
    s = math.sqrt(rx * rx + ry * ry + rz * rz)
    c = cx * nx + cy * ny + cz * nz
    angle_deg = math.degrees(math.atan2(s, c))
    if s < 1e-9:
        return (0.0, 0.0, 0.0), angle_deg
    k = angle_deg / s
    return (rx * k, ry * k, rz * k), angle_deg


def tilt_to_unit_command(tilt_deg):
    """데드존 + 선형 램프: |tilt| <= deadzone -> 0, >= full -> +-1."""
    a = abs(tilt_deg)
    if a <= WHEEL_TILT_DEADZONE_DEG:
        return 0.0
    frac = (a - WHEEL_TILT_DEADZONE_DEG) / (WHEEL_TILT_FULL_DEG - WHEEL_TILT_DEADZONE_DEG)
    return math.copysign(min(1.0, frac), tilt_deg)


def stick_axis_unit(raw, center, below, above):
    """스틱 원값 한 축 -> -1..1 (캘리브레이션의 중심/아래 폭/위 폭 기준)."""
    d = raw - center
    return max(-1.0, min(1.0, d / (above if d > 0 else below)))


def stick_to_rate(u):
    """데드존 + 선형 램프: |u| <= STICK_DEADZONE -> 0, >= STICK_FULL -> +-1."""
    a = abs(u)
    if a <= STICK_DEADZONE:
        return 0.0
    return math.copysign(min(1.0, (a - STICK_DEADZONE) / (STICK_FULL - STICK_DEADZONE)), u)


def decode_stick_calibration(side, raw):
    """Joy-Con SPI의 스틱 캘리브레이션 9바이트 (12비트 값 6개) -> {"center",
    "below", "above"} (각각 (x, y)), 값이 말이 안 되면 None. 순서는 좌우가
    다르다: 왼쪽 = 위 폭, 중심, 아래 폭; 오른쪽 = 중심, 아래 폭, 위 폭."""
    b = raw
    v = [
        ((b[1] << 8) & 0xF00) | b[0],
        (b[2] << 4) | (b[1] >> 4),
        ((b[4] << 8) & 0xF00) | b[3],
        (b[5] << 4) | (b[4] >> 4),
        ((b[7] << 8) & 0xF00) | b[6],
        (b[8] << 4) | (b[7] >> 4),
    ]
    if side == "left":
        above, center, below = (v[0], v[1]), (v[2], v[3]), (v[4], v[5])
    else:
        center, below, above = (v[0], v[1]), (v[2], v[3]), (v[4], v[5])
    if not all(1000 <= c <= 3100 for c in center) or not all(300 <= s <= 2100 for s in below + above):
        return None
    return {"center": center, "below": below, "above": above}


_JOYCON_RUMBLE_NEUTRAL = b"\x00\x01\x40\x40\x00\x01\x40\x40"


def _spi_read_validated(dev, address, size, packet_number):
    """원시 hidapi 핸들로 SPI 플래시를 읽되, 정말로 이 요청에 대한 응답인
    0x21 리플라이 (subcommand 0x10, 같은 address와 size)만 받아들인다 --
    joyconrobotics 자체의 _send_subcmd_get_response()는 처음 보이는 0x21을
    그냥 가져가는데, 그게 아래에서 우회하는 버그다."""
    packet = (
        b"\x01" + bytes([packet_number & 0xF]) + _JOYCON_RUMBLE_NEUTRAL
        + b"\x10" + address.to_bytes(4, "little") + bytes([size])
    )
    for _ in range(5):
        dev.write(packet)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            r = bytes(dev.read(64, 50))
            if (
                len(r) >= 20 + size and r[0] == 0x21 and r[13] & 0x80 and r[14] == 0x10
                and int.from_bytes(r[15:19], "little") == address and r[19] == size
            ):
                return r[20:20 + size]
    raise IOError(f"no valid SPI reply for {address:#06x}")


def _read_stick_calibration(dev, side):
    """사용자 스틱 캘리브레이션이 있고 말이 되면 그것, 아니면 공장값. 둘 다
    안 되면 None."""
    magic_addr, user_addr, factory_addr = (0x8010, 0x8012, 0x603D) if side == "left" else (0x801B, 0x801D, 0x6046)
    if _spi_read_validated(dev, magic_addr, 2, 3) == b"\xb2\xa1":
        cal = decode_stick_calibration(side, _spi_read_validated(dev, user_addr, 9, 4))
        if cal is not None:
            return cal
    return decode_stick_calibration(side, _spi_read_validated(dev, factory_addr, 9, 5))


def read_joycon_calibration(side):
    """(IMU 캘리브레이션 24바이트, 스틱 캘리브레이션 dict)을 우리가 따로 잠깐
    연 핸들로 응답 검증하며 읽는다. 각각 못 읽으면 None.
    IMU는 사용자 캘리브레이션이 있으면 그것, 없으면 공장값 (joyconrobotics와
    같은 선택); None이면 joyconrobotics 자체 값을 검사 없이 사용."""
    import hid

    vid, pid, _ = (_joycon_device.get_L_id if side == "left" else _joycon_device.get_R_id)()
    if vid is None:
        return None, None  # 해당 Joy-Con 없음 -- joyconrobotics가 자체 에러를 냄
    imu = stick = None
    try:
        # `hid`라는 이름의 파이썬 모듈이 두 종류 있고 joyconrobotics는 둘 다
        # 받아들인다 (joycon.py _open): cython-hidapi는 hid.device() + open(),
        # ctypes 기반 "hid" 패키지는 hid.Device(vid, pid). 둘 다
        # write(bytes)와 read(size, timeout_ms)를 가진다.
        if hasattr(hid, "device"):
            dev = hid.device()
            dev.open(vid, pid)
        else:
            dev = hid.Device(vid, pid)
        try:
            try:
                user = _spi_read_validated(dev, 0x8026, 2, 1)
                imu = _spi_read_validated(dev, 0x8028 if user == b"\xb2\xa1" else 0x6020, 24, 2)
            except Exception as e:
                print(f"[{side}] couldn't read the IMU calibration for checking ({e!r}) -- using joyconrobotics' values unchecked")
            try:
                stick = _read_stick_calibration(dev, side)
            except Exception as e:
                print(f"[{side}] couldn't read the stick calibration ({e!r})")
        finally:
            dev.close()
    except Exception as e:
        print(f"[{side}] couldn't open the Joy-Con to read its calibration ({e!r}) -- using joyconrobotics' IMU values unchecked")
    return imu, stick


class JointNudgeJoyconRobotics(JoyconRobotics):
    def __init__(
        self,
        *args,
        nudge_deg_per_tick=NUDGE_DEG_PER_TICK,
        motor1_limit_deg=80.0,
        motor2_limit_deg=80.0,
        motor3_limit_deg=80.0,
        wrist_flex_limit_deg=WRIST_FLEX_LIMIT_DEG,
        wrist_roll_limit_deg=WRIST_ROLL_LIMIT_DEG,
        **kwargs,
    ):
        self.motor1_delta = 0.0  # shoulder_pan
        self.motor2_delta = 0.0  # shoulder_lift
        self.motor3_delta = 0.0  # elbow_flex
        self.wrist_flex_deg = 0.0  # 자이로 적분값, orientation_rad에서 오지 않음 -- 모듈 docstring 참고
        self.wrist_roll_deg = 0.0  # 마찬가지로 자이로 적분값 (gx) -- 모듈 docstring 참고
        self.gripper_deg = GRIPPER_CLOSED_DEG  # 그리퍼 목표 (도); Teleop이 시작 시 다시 맞춤
        self.gripper_opening = GRIPPER_START_OPENING  # 누르고 있을 때 움직일 방향 (True = 열기)
        self.gripper_btn_prev = 0  # 직전 틱의 ZR/ZL 상태 (누름 순간 감지용)
        self.reset_btn_prev = 0  # 직전 틱의 plus/minus 상태 (누름 순간 감지용)
        self.nudge_deg_per_tick = nudge_deg_per_tick
        self.motor1_limit_deg = motor1_limit_deg
        self.motor2_limit_deg = motor2_limit_deg
        self.motor3_limit_deg = motor3_limit_deg
        self.wrist_flex_limit_deg = wrist_flex_limit_deg
        self.wrist_roll_limit_deg = wrist_roll_limit_deg
        self.stick_unit = (0.0, 0.0)  # 마지막 스틱 값 (-1..1, 가로/세로), 디버그 출력용
        self.stick_click_t = -math.inf  # 스틱 클릭이 마지막으로 눌려 있던 시각
        self.stick_live = False  # Teleop이 ARM 진입 때 False로; 스틱이 데드존 안에 들어오면 True
        self.camera_stick = False  # True면 R을 누르는 동안 이 스틱은 팔 대신 카메라 (Teleop이 정함)
        # args[0] / kwargs["device"]가 joyconrobotics의 좌/우 선택자
        # ("right"/"left"); WRIST_GYRO_SIGN 조회용으로 기억해 둔다.
        self.side = kwargs.get("device", args[0] if args else "right")
        self.roll_sign, self.flex_sign = WRIST_GYRO_SIGN[self.side]
        # Step 2 상태. 전부 super().__init__() 전에 만들어져 있어야 한다.
        # joyconrobotics가 common_update()를 호출하는 solve_loop 스레드를
        # 거기서 시작하기 때문.
        # None = 비활성: Teleop이 팔 상태를 리셋하고 ARM으로 전환할 때까지
        # nudge / 손목 적분 / 그리퍼 토글을 하지 않는다. 그렇지 않으면 두 번째
        # Joy-Con이 캘리브레이션하고 팔이 0으로 가는 동안 첫 번째 Joy-Con이
        # 적분을 해버리고 (예: 책상에서 집어 들 때), 첫 원격조작 틱에서 팔이
        # 튀게 된다.
        self.mode = None  # Teleop (메인 스레드)만 씀
        self.l3_presses = 0  # 누름 카운터: 여기서 증가만 한다
        self.zl_presses = 0
        self.head_recenter_presses = 0
        self.wrist_reset_presses = 0  # Teleop이 로그를 찍는 용도
        self.accel_filtered = None  # 로우패스한 가속도 (g), 튜플; Teleop이 읽음
        self._accel_last_t = None
        self._report_times = [None, None, None]  # 장치 핸들별 최신 리포트 시각
        # 2026-09-30, 실제 Joy-Con에서 발견: joyconrobotics는 Joy-Con 하나를
        # 세 번 열고, 모든 핸들이 모든 0x21 리플라이를 받는다 -- 이전 핸들의
        # 센서 설정 서브커맨드에 대한 ACK까지 포함해서. 그쪽 SPI 읽기는 처음
        # 보이는 0x21을 가져가므로, 나중 핸들이 IMU 캘리브레이션을 리플라이
        # 하나 밀린 채로 로드할 수 있다 (실제로 본 것: 왼쪽 accel X = 144 g,
        # gyro X = 3e7 rad/s -- 양쪽 어디서든, 무작위로). 그래서 먼저 진짜
        # 캘리브레이션을 직접 읽고; 나중에 IMU 데이터를 실제로 쓰는 핸들
        # (self.gyro)을 확인 (필요하면 수리)한다.
        imu_cal, stick_cal = read_joycon_calibration(self.side)
        if stick_cal is None:
            print(f"[{self.side}] no usable stick calibration on the Joy-Con -- using a generic one")
            stick_cal = STICK_FALLBACK_CAL
        self.stick_cal = stick_cal
        super().__init__(*args, **kwargs)
        # joyconrobotics는 Joy-Con을 세 번 (joycon/gyro/button) 열고, 각각
        # 자기 리더 스레드를 가진다; 세 핸들 모두에서 리포트마다 타임스탬프를
        # 찍는다. 훅은 리더 스레드에서 돌고 절대 예외를 던지면 안 된다
        # (거기서 예외가 나면 그 리더가 영영 죽는다).
        for idx, device in enumerate((self.joycon, self.gyro, self.button)):
            device.register_update_hook(self._make_report_hook(idx))
        if imu_cal is not None:
            self._repair_imu_calibration(imu_cal)

    def _repair_imu_calibration(self, cal):
        """self.gyro에 로드된 캘리브레이션을 검증된 블록과 비교한다;
        다르면 올바른 값을 로드하고 자이로 바이어스를 다시 측정한다
        (joyconrobotics 생성자 안의 2초 바이어스 측정은 잘못된 스케일
        계수로 이뤄졌으므로)."""

        def i16(k):
            return int.from_bytes(cal[k:k + 2], "little", signed=True)

        dev = self.gyro
        accel_off = (i16(0), i16(2), i16(4))
        accel_coeff = tuple(0x4000 / c if (c != 0x4000 and c != 0) else 1 for c in (i16(6), i16(8), i16(10)))
        gyro_coeff = tuple(
            0x343B / c if (c != 0x343B and c != 0 and dev.calibrate_value) else 1 for c in (i16(18), i16(20), i16(22))
        )
        loaded = (
            (dev._ACCEL_OFFSET_X, dev._ACCEL_OFFSET_Y, dev._ACCEL_OFFSET_Z),
            (dev._ACCEL_COEFF_X, dev._ACCEL_COEFF_Y, dev._ACCEL_COEFF_Z),
            (dev._GYRO_COEFF_X, dev._GYRO_COEFF_Y, dev._GYRO_COEFF_Z),
        )
        expected = (accel_off, accel_coeff, gyro_coeff)
        if all(abs(a - b) < 1e-9 for got, want in zip(loaded, expected) for a, b in zip(got, want)):
            return  # 제대로 로드됨 (생성자에서 측정한 바이어스는 그대로 유지)
        print(
            f"[{self.side}] joyconrobotics loaded a corrupted IMU calibration (known reply mix-up) -- "
            "fixed; re-measuring the gyro bias, keep the Joy-Con still ~2s..."
        )
        dev.set_accel_calibration(accel_off, (i16(6), i16(8), i16(10)))
        dev.set_gyro_calibration((i16(12), i16(14), i16(16)), (i16(18), i16(20), i16(22)))
        dev.calibrate(2)
        deadline = time.monotonic() + 4.0
        while dev.is_calibrating and time.monotonic() < deadline:
            time.sleep(0.05)
        # 가속도계 로우패스를 새로 들어오는 (올바르게 스케일된) 데이터로 다시 시작
        self.accel_filtered = None
        self._accel_last_t = None
        print(f"[{self.side}] IMU calibration repaired.")

    def _make_report_hook(self, idx):
        times = self._report_times

        def hook(_device):
            times[idx] = time.monotonic()

        return hook

    def report_age(self, now=None):
        """세 장치 핸들 중 가장 오래된 것이 마지막으로 입력 리포트를 준 뒤
        지난 초 (세 개 모두 한 번씩 받기 전까지는 inf)."""
        if now is None:
            now = time.monotonic()
        times = list(self._report_times)
        if any(t is None for t in times):
            return math.inf
        return now - min(times)

    def read_stick(self):
        """이 Joy-Con 스틱의 (가로, 세로), 각각 -1..1. + = 오른쪽 / 위
        (Joy-Con을 세로로 쥐었을 때, 버튼 라벨과 같은 기준)."""
        if self.joycon.is_right():
            raw = (self.joycon.get_stick_right_horizontal(), self.joycon.get_stick_right_vertical())
        else:
            raw = (self.joycon.get_stick_left_horizontal(), self.joycon.get_stick_left_vertical())
        cal = self.stick_cal
        return tuple(stick_axis_unit(raw[i], cal["center"][i], cal["below"][i], cal["above"][i]) for i in (0, 1))

    def _update_accel_filter(self, now):
        samples = self.gyro.accel_in_g  # 리포트당 샘플 3개, g 단위 (x, y, z)
        ax = (samples[0][0] + samples[1][0] + samples[2][0]) / 3.0
        ay = (samples[0][1] + samples[1][1] + samples[2][1]) / 3.0
        az = (samples[0][2] + samples[1][2] + samples[2][2]) / 3.0
        prev = self.accel_filtered
        if prev is None or self._accel_last_t is None:
            self.accel_filtered = (ax, ay, az)
        else:
            dt = min(max(now - self._accel_last_t, 0.0), 0.1)
            alpha = 1.0 - math.exp(-dt / ACCEL_LPF_TAU_S)
            self.accel_filtered = (
                prev[0] + alpha * (ax - prev[0]),
                prev[1] + alpha * (ay - prev[1]),
                prev[2] + alpha * (az - prev[2]),
            )
        self._accel_last_t = now

    def common_update(self):
        is_right = self.joycon.is_right()
        step = self.nudge_deg_per_tick
        mode = self.mode
        now = time.monotonic()
        fresh = self.report_age(now) <= JOYCON_WATCHDOG_S
        if fresh:
            self._update_accel_filter(now)
        # 팔 입력은 ARM 모드에서만, 그리고 살아있는 데이터일 때만: 블루투스
        # 연결이 끊기면 마지막 리포트가 (버튼이 눌린 채거나 자이로 각속도가
        # 0이 아닌 채로) 그대로 남는데, 그러면 영원히 nudge / 적분을 계속하게
        # 된다. WHEEL 모드에서는 delta와 손목 적분기를 그냥 건드리지 않으므로,
        # 팔은 자세를 유지하다가 ARM 모드로 돌아오면 거기서부터 이어간다.
        arm_active = fresh and mode == MODE_ARM

        # 스틱 클릭 시각은 모드와 무관하게 기록한다 (STICK_CLICK_HOLDOFF_S 참고).
        click = self.joycon.get_button_r_stick() if is_right else self.joycon.get_button_l_stick()
        if click == 1:
            self.stick_click_t = now

        # 그리퍼 버튼의 직전 상태는 모드와 무관하게 갱신한다 -- WHEEL 모드에서
        # 누른 채 ARM으로 돌아와도 "새로 누름"으로 잘못 세지 않게.
        gripper_held = self.joycon.get_button_zr() if is_right else self.joycon.get_button_zl()
        gripper_pressed = gripper_held == 1 and self.gripper_btn_prev != 1
        self.gripper_btn_prev = gripper_held

        if arm_active:
            # 2026-10-01: 모터 1/2를 버튼 (Y/A, B/X / 화살표) 대신 스틱으로
            # (사용자 요청). 예전 버튼 방향: A/오른쪽 = pan +, B/아래 = lift +.
            # 시작할 때와 ARM으로 돌아올 때마다 스틱은 한 번 데드존 안으로
            # 들어와야 살아난다 (stick_live) -- 기울인 채로 시작해도 팔이 가지 않게.
            sx, sy = self.read_stick()
            self.stick_unit = (sx, sy)
            rate_x, rate_y = stick_to_rate(sx), stick_to_rate(sy)
            if is_right and self.camera_stick and self.joycon.get_button_r() == 1:
                self.stick_live = False  # R+스틱 = 카메라 (Teleop이 처리); 뗀 뒤엔 다시 가운데부터
            elif not self.stick_live:
                self.stick_live = rate_x == 0.0 and rate_y == 0.0
            elif now - self.stick_click_t >= STICK_CLICK_HOLDOFF_S:
                self.motor1_delta += step * STICK_PAN_SIGN * rate_x
                self.motor2_delta += step * STICK_LIFT_SIGN * rate_y

            # 2026-10-01: 모터 3을 L/ZL (R/ZR)에서 위/아래 화살표 (X/B)로 옮김
            # (사용자 요청). 위치를 그대로 따름: 위쪽 버튼 L (R) -> 위 (X) = -,
            # 아래쪽 버튼 ZL (ZR) -> 아래 (B) = + (2026-09-04에 정한 방향 유지).
            elbow_pos = self.joycon.get_button_b() if is_right else self.joycon.get_button_down()
            elbow_neg = self.joycon.get_button_x() if is_right else self.joycon.get_button_up()
            if elbow_pos == 1:
                self.motor3_delta += step
            elif elbow_neg == 1:
                self.motor3_delta -= step

            # 그리퍼 (공식 예제 방식): 새로 누를 때마다 방향을 바꾸고, 누르고
            # 있는 동안만 그 방향의 끝 쪽으로 (GRIPPER_* 참고).
            if gripper_pressed:
                self.gripper_opening = not self.gripper_opening
            if gripper_held == 1:
                end = GRIPPER_OPEN_DEG if self.gripper_opening else GRIPPER_CLOSED_DEG
                if self.gripper_deg < end:
                    self.gripper_deg = min(end, self.gripper_deg + GRIPPER_DEG_PER_TICK)
                else:
                    self.gripper_deg = max(end, self.gripper_deg - GRIPPER_DEG_PER_TICK)

        self.motor1_delta = max(-self.motor1_limit_deg, min(self.motor1_limit_deg, self.motor1_delta))
        self.motor2_delta = max(-self.motor2_limit_deg, min(self.motor2_limit_deg, self.motor2_delta))
        self.motor3_delta = max(-self.motor3_limit_deg, min(self.motor3_limit_deg, self.motor3_delta))

        if arm_active:
            # wrist_flex/wrist_roll: 원시 자이로 누설 적분기, orientation_rad를
            # 완전히 우회 -- 이유는 모듈 docstring 참고 (flex: 이 파지에서
            # 가속도계 pitch 포화; roll: flex에서 오는 az 기인 크로스토크).
            # gx/gy는 gyro_in_rad의 인덱스 0/1 (AttitudeEstimator.update()
            # 자체의 언패킹과 일치) -- roll엔 gx, flex엔 gy를 쓰면 둘이
            # 수학적으로 독립으로 유지된다.
            gx, gy, _ = self.gyro.gyro_in_rad[0]
            gx *= self.roll_sign
            gy *= self.flex_sign
            self.wrist_flex_deg = (
                self.wrist_flex_deg * WRIST_FLEX_DECAY_PER_TICK - gy * WRIST_FLEX_GYRO_DT * WRIST_FLEX_GAIN_DEG_PER_RAD
            )
            self.wrist_flex_deg = max(-self.wrist_flex_limit_deg, min(self.wrist_flex_limit_deg, self.wrist_flex_deg))

            self.wrist_roll_deg = (
                self.wrist_roll_deg * WRIST_ROLL_DECAY_PER_TICK - gx * WRIST_ROLL_GYRO_DT * WRIST_ROLL_GAIN_DEG_PER_RAD
            )
            self.wrist_roll_deg = max(-self.wrist_roll_limit_deg, min(self.wrist_roll_limit_deg, self.wrist_roll_deg))

        # 2026-09-05: home/plus 분리 (예전엔 둘 중 아무거나 그리퍼를 토글).
        # home (오른쪽) / capture (왼쪽) -> 그리퍼 전용 (2026-10-01부터 누르고
        # 있는 동안만 움직임, 위에서 처리). plus (오른쪽) /
        # minus (왼쪽) -> wrist_flex/roll을 0 (자이로 적분 원점)으로 복귀.
        # 2026-10-01: R (오른쪽) / L (왼쪽) -> 그리퍼 방향 (닫기 <-> 열기) 전환.
        # (2026-10-01 오후: 없앰 -- 방향은 그리퍼 버튼을 새로 누를 때 바뀜.
        # 같은 날 저녁: 그리퍼 버튼 자체가 ZR/ZL로 옮겨감, home/capture는 빔.)
        # 이 둘은 이제 절대 센서값이 아니라 범위 제한이 느슨하고 드리프트하기
        # 쉬운 적분기이므로 -- 수동 원점 복귀 버튼이 누설 감쇠를 기다리지 않고
        # 누적 오차를 빠르게 바로잡는 방법이다.
        # 2026-09-29: 이것들은 ARM 모드에서만 동작한다 (WHEEL 모드에선 팔이
        # 고정됨); WHEEL 모드에서는 오른쪽 plus가 대신 헤드 카메라를 원점
        # 복귀시킨다. L3는 모드와 무관하게 카운트한다 (Teleop이 토글).
        # 왼쪽 ZL은 WHEEL 모드에서만 브레이크용으로 센다 -- ARM 모드에서는
        # 그리퍼 버튼이라, 거기서 센 누름이 WHEEL로 넘어가 브레이크를 걸면 안 된다.
        # 2026-10-06: plus/minus는 이벤트 버퍼(self.button.events())가 아니라
        # 버튼 상태를 직접 읽는다 (elbow 버튼과 같은 self.joycon). 1001까지는
        # 이벤트로 받았는데 실기에서 눌러도 손목이 돌아오지 않았다 (사용자
        # 보고; 0930의 R3 간헐 실패도 같은 오른쪽 이벤트 경로였다). 원인은
        # 확인하지 못했고, 실기에서 잘 된 직접 읽기 방식으로 옮겼다.
        # 동작은 그대로: ARM이면 손목 적분기를 0 (시작 자세)으로, WHEEL이면
        # 오른쪽 plus = 헤드 카메라 중앙.
        reset_held = self.joycon.get_button_plus() if is_right else self.joycon.get_button_minus()
        reset_pressed = fresh and reset_held == 1 and self.reset_btn_prev != 1
        self.reset_btn_prev = reset_held
        if reset_pressed:
            if mode == MODE_ARM:
                self.wrist_flex_deg = 0.0
                self.wrist_roll_deg = 0.0
                self.wrist_reset_presses += 1
            elif mode == MODE_WHEEL and is_right:
                self.head_recenter_presses += 1

        for event_type, status in self.button.events():
            if not is_right and event_type == "stick_l_btn":
                if status == 1:
                    self.l3_presses += 1
            elif not is_right and event_type == "zl":
                if status == 1 and mode == MODE_WHEEL:
                    self.zl_presses += 1

        return self.position, self.gripper_state, self.button_control


class ArmBinding:
    """Joy-Con 하나가 팔 하나를 구동. `prefix`는 로봇 클래스가 관절 이름 앞에
    붙이는 것 (SO101Follower는 "", XLerobot2Wheels는 "left_arm_"/"right_arm_");
    `jc`는 그 손의 JointNudgeJoyconRobotics."""

    def __init__(self, side, prefix, jc):
        self.side = side
        self.prefix = prefix
        self.jc = jc
        self.base = {}  # 원격조작 시작 시점의 팔 pan/lift/elbow (delta가 여기에 더해짐)

    def key(self, joint):
        return f"{self.prefix}{joint}"

    def capture_base(self, current):
        self.base = {j: current[self.key(j)] for j in ("shoulder_pan", "shoulder_lift", "elbow_flex")}
        print(
            f"[{self.side}] arm start: pan={self.base['shoulder_pan']:.1f} "
            f"lift={self.base['shoulder_lift']:.1f} elbow={self.base['elbow_flex']:.1f}"
        )

    def targets(self):
        """이 Joy-Con을 읽어서 {접두사 붙은 관절: 목표 도}를 반환."""
        jc = self.jc
        return {
            self.key("shoulder_pan"): self.base["shoulder_pan"] + jc.motor1_delta,
            self.key("shoulder_lift"): self.base["shoulder_lift"] + jc.motor2_delta,
            self.key("elbow_flex"): self.base["elbow_flex"] + jc.motor3_delta,
            self.key("wrist_flex"): jc.wrist_flex_deg,
            self.key("wrist_roll"): jc.wrist_roll_deg,
            self.key("gripper"): jc.gripper_deg,
        }

    def debug_str(self):
        jc = self.jc
        sx, sy = jc.stick_unit
        return (
            f"{self.side}: pan={jc.motor1_delta:+.1f} lift={jc.motor2_delta:+.1f} "
            f"elbow={jc.motor3_delta:+.1f} flex={jc.wrist_flex_deg:+.1f} roll={jc.wrist_roll_deg:+.1f} "
            f"grip={jc.gripper_deg:.1f}({'open' if jc.gripper_opening else 'close'}) stick=({sx:+.2f},{sy:+.2f})"
        )


class TiltDrive:
    """왼쪽 Joy-Con 기울기 -> (x.vel m/s, theta.vel deg/s). rearm() 후
    WHEEL_NEUTRAL_SETTLE_S 지나서 잡는 중립 자세, 데드존/램프, 기울기 컷오프,
    래치형 브레이크를 포함. 순수 로직 -- `now`는 인자로 받는다."""

    def __init__(self):
        self.brake = False
        self.neutral = None  # 중립 자세에서의 중력 단위벡터
        self.settle_until = 0.0
        self.tilt_fwd = 0.0  # 마지막으로 계산한 기울기 (도), 디버그 출력용
        self.tilt_turn = 0.0
        self.tilt_total = 0.0

    def rearm(self, now):
        """중립 자세를 잊고; 안정되면 새로 잡는다."""
        self.neutral = None
        self.settle_until = now + WHEEL_NEUTRAL_SETTLE_S

    def release(self, now):
        self.brake = False
        self.rearm(now)

    def command(self, accel_g, now):
        self.tilt_fwd = self.tilt_turn = self.tilt_total = 0.0
        g = gravity_unit(accel_g)
        if g is None:
            return 0.0, 0.0
        if self.neutral is None:
            if now >= self.settle_until:
                self.neutral = g
            return 0.0, 0.0
        rot, total = tilt_rotation_deg(self.neutral, g)
        self.tilt_fwd = WHEEL_FORWARD_SIGN * rot[WHEEL_FORWARD_AXIS]
        self.tilt_turn = WHEEL_TURN_SIGN * rot[WHEEL_TURN_AXIS]
        self.tilt_total = total
        if self.brake or total > WHEEL_TILT_CUTOFF_DEG:
            return 0.0, 0.0
        return (
            tilt_to_unit_command(self.tilt_fwd) * WHEEL_MAX_LINEAR_MPS,
            tilt_to_unit_command(self.tilt_turn) * WHEEL_MAX_ANGULAR_DEGPS,
        )


class HeadCamera:
    """헤드 (중앙 카메라) pan/tilt 목표 각도 (도), 버튼으로 nudge.
    헤드의 현재 자세에서 시작하고 (시작 시 아무것도 안 움직임); nudge는
    캘리브레이션된 반범위 - JOINT_LIMIT_MARGIN_DEG로 클램프된다
    (이미 범위 밖에 있는 목표는 안쪽으로만 움직일 수 있다)."""

    MOTORS = (HEAD_TILT_MOTOR, HEAD_PAN_MOTOR)

    def __init__(self, calibration, current):
        self.limits = {m: joint_half_range_deg(calibration, m) for m in self.MOTORS}
        self.target = {m: current[m] for m in self.MOTORS}

    def _nudge(self, motor, delta):
        old = self.target[motor]
        new = old + delta
        limit = self.limits[motor]
        if abs(new) > limit:
            if abs(old) <= limit:
                new = math.copysign(limit, new)
            elif abs(new) > abs(old):
                return
        self.target[motor] = new

    def update(self, up, down, left, right):
        step = HEAD_NUDGE_DEG_PER_TICK
        if up and not down:
            self._nudge(HEAD_TILT_MOTOR, step * HEAD_TILT_UP_SIGN)
        elif down and not up:
            self._nudge(HEAD_TILT_MOTOR, -step * HEAD_TILT_UP_SIGN)
        if left and not right:
            self._nudge(HEAD_PAN_MOTOR, step * HEAD_PAN_LEFT_SIGN)
        elif right and not left:
            self._nudge(HEAD_PAN_MOTOR, -step * HEAD_PAN_LEFT_SIGN)

    def drive(self, right_rate, up_rate):
        """스틱용: 각각 -1..1 (+ = 오른쪽 / 위), 비례 속도."""
        step = HEAD_NUDGE_DEG_PER_TICK
        if up_rate:
            self._nudge(HEAD_TILT_MOTOR, step * up_rate * HEAD_TILT_UP_SIGN)
        if right_rate:
            self._nudge(HEAD_PAN_MOTOR, -step * right_rate * HEAD_PAN_LEFT_SIGN)

    def recenter(self):
        for m in self.MOTORS:
            self.target[m] = 0.0

    def targets(self):
        return dict(self.target)


class Teleop:
    """제어 틱 한 번 = step(now): 모드 전환, 바퀴/헤드 로직, 워치독, 그다음
    모든 것에 대해 get_observation() + send_action() 한 번씩."""

    def __init__(self, robot, arms, *, has_base, wheel_dry_run, current):
        self.robot = robot
        self.arms = arms
        self.jcs = {arm.side: arm.jc for arm in arms}
        self.has_base = has_base
        # WHEEL 모드는 두 손이 다 필요하다: 왼쪽이 주행과 L3 (모드 토글),
        # 오른쪽에는 카메라 버튼이 있다.
        self.wheel_mode_available = has_base and "left" in self.jcs and "right" in self.jcs
        self.wheel_dry_run = wheel_dry_run
        self.mode = MODE_ARM
        self._last_toggle_t = -math.inf
        self.drive = TiltDrive()
        self.head = HeadCamera(robot.calibration, current) if self.wheel_mode_available else None
        self.tick = 0
        self.last_cmd = (0.0, 0.0)
        self._stale = set()
        self._leds_enabled = self.wheel_mode_available
        self._led_state = None
        # 모든 팔을 깨끗한 상태에서 시작시키고 (jc.mode가 None인 동안 Joy-Con
        # 스레드는 이 값들을 건드리지 않음), 그다음에 활성화한다. 이 시점 전에
        # 눌린 것 (예: move-to-zero 도중)은 무시된다.
        for jc in self.jcs.values():
            jc.motor1_delta = jc.motor2_delta = jc.motor3_delta = 0.0
            jc.wrist_flex_deg = jc.wrist_roll_deg = 0.0
            jc.gripper_deg = GRIPPER_CLOSED_DEG  # move-to-zero가 그리퍼를 0 (닫힘)으로 보냈다
            jc.gripper_opening = GRIPPER_START_OPENING
            jc.stick_live = False
            jc.camera_stick = self.head is not None and jc is self.jcs.get("right")
        self._seen = {(side, attr): getattr(jc, attr) for side, jc in self.jcs.items() for attr in PRESS_COUNTERS}
        self._gripper_opening_seen = {side: jc.gripper_opening for side, jc in self.jcs.items()}
        for side, jc in self.jcs.items():
            sx, sy = jc.read_stick()
            if stick_to_rate(sx) or stick_to_rate(sy):
                print(
                    f"[{side}] WARNING: stick reads ({sx:+.2f}, {sy:+.2f}) while it should be untouched -- "
                    "outside the dead zone. The stick stays ignored until you let go of it; if pan/lift "
                    "never respond, the stick is drifting: raise STICK_DEADZONE."
                )
        for jc in self.jcs.values():
            jc.mode = MODE_ARM
        self._update_leds()

    def _presses(self, side, attr):
        jc = self.jcs.get(side)
        if jc is None:
            return 0
        value = getattr(jc, attr)
        new = value - self._seen[(side, attr)]
        self._seen[(side, attr)] = value
        return new

    def _set_mode(self, new_mode, now):
        if new_mode == self.mode:
            return
        self.mode = new_mode
        for jc in self.jcs.values():
            if new_mode == MODE_ARM:
                jc.stick_live = False
            jc.mode = new_mode
        if new_mode == MODE_WHEEL:
            self.drive.release(now)  # 브레이크 해제, 안정된 뒤 중립 다시 잡음
            print(
                "[mode] WHEEL -- arms frozen. Left tilt fwd/back = drive, left/right = turn, "
                "ZL = brake, right X/B/Y/A or R+stick = camera, plus = camera center, L3 = back to ARM."
            )
        else:
            print("[mode] ARM -- wheels stopped, arms live again. L3 = WHEEL mode.")

    def _report_stale_changes(self, stale):
        for side in sorted(stale - self._stale):
            print(
                f"[watchdog] {side} Joy-Con: no input for >{JOYCON_WATCHDOG_S:.1f}s -- its input is ignored. "
                "A short stall recovers by itself; if it really disconnected, re-pairing does NOT help "
                "(joyconrobotics never reopens it) -- Ctrl+C and restart."
            )
        for side in sorted(self._stale - stale):
            print(f"[watchdog] {side} Joy-Con: input is back")
        self._stale = stale

    def _update_leds(self):
        if not self._leds_enabled:
            return
        if self.mode == MODE_WHEEL:
            state = "brake" if self.drive.brake else "wheel"
        else:
            state = "arm"
        if state == self._led_state:
            return
        self._led_state = state
        for jc in self.jcs.values():
            try:
                if state == "brake":
                    jc.joycon.set_player_lamp_flashing(LED_WHEEL_PATTERN)
                elif state == "wheel":
                    jc.joycon.set_player_lamp_on(LED_WHEEL_PATTERN)
                else:
                    jc.joycon.set_player_lamp_on(LED_ARM_PATTERN)
            except Exception as e:  # LED는 장식일 뿐 -- 절대 원격조작을 멈추게 두지 않음
                self._leds_enabled = False
                print(f"[led] couldn't set Joy-Con LEDs ({e!r}); LED mode display disabled, control unaffected")
                return

    def step(self, now):
        self.tick += 1
        l3 = self._presses("left", "l3_presses")
        zl = self._presses("left", "zl_presses")
        head_recenter = self._presses("right", "head_recenter_presses")
        for side in self.jcs:
            if self._presses(side, "wrist_reset_presses"):
                button = "plus" if side == "right" else "minus"
                print(f"[{side}] wrist reset ({button}) -> flex/roll 0")

        stale = {side for side, jc in self.jcs.items() if jc.report_age(now) > JOYCON_WATCHDOG_S}
        self._report_stale_changes(stale)

        for side, jc in self.jcs.items():
            opening = jc.gripper_opening
            if opening != self._gripper_opening_seen[side]:
                self._gripper_opening_seen[side] = opening
                hold = "ZR" if side == "right" else "ZL"
                print(f"[{side}] gripper: {'OPEN' if opening else 'CLOSE'} direction (hold {hold})")

        switched = False
        if self.wheel_mode_available and l3:
            # 2026-10-01 저녁 (사용자 요청): L3 하나로 ARM <-> WHEEL 토글.
            # 예전의 "L3 = WHEEL, R3 = ARM" 절대 지정과 L3+R3 동시 처리는 없앴다.
            if now - self._last_toggle_t < MODE_TOGGLE_MIN_INTERVAL_S:
                print("[mode] L3 ignored -- pressed again too quickly (counted once)")
            else:
                self._last_toggle_t = now
                switched = True
                self._set_mode(MODE_WHEEL if self.mode == MODE_ARM else MODE_ARM, now)

        x_vel = theta_vel = 0.0
        if self.mode == MODE_WHEEL:
            if zl and not switched:  # 모드를 바꾼 틱의 ZL은 무시 (진입하자마자 브레이크 방지)
                if self.drive.brake:
                    self.drive.release(now)
                    print("[wheel] brake OFF -- hold the Joy-Con at its neutral pose for a moment")
                else:
                    self.drive.brake = True
                    print("[wheel] brake ON")
            if stale and not self.drive.brake:
                self.drive.brake = True
                print("[watchdog] wheels BRAKED (latched) -- ZL releases it if the stall clears; after a real disconnect, Ctrl+C and restart")
            x_vel, theta_vel = self.drive.command(self.jcs["left"].accel_filtered, now)
            if "right" not in stale:
                if head_recenter:
                    self.head.recenter()
                    print("[head] camera center (plus)")
                buttons = self.jcs["right"].joycon
                self.head.update(
                    up=buttons.get_button_x() == 1,
                    down=buttons.get_button_b() == 1,
                    left=buttons.get_button_y() == 1,
                    right=buttons.get_button_a() == 1,
                )
        self.last_cmd = (x_vel, theta_vel)

        # R + 오른쪽 스틱 -> 카메라, 모드와 무관 (HEAD_NUDGE_DEG_PER_TICK 아래 메모 참고).
        if self.head is not None and "right" not in stale:
            right = self.jcs["right"]
            if right.joycon.get_button_r() == 1 and now - right.stick_click_t >= STICK_CLICK_HOLDOFF_S:
                sx, sy = right.read_stick()
                self.head.drive(stick_to_rate(sx), stick_to_rate(sy))

        targets = {}
        for arm in self.arms:
            targets.update(arm.targets())
        if self.head is not None:
            targets.update(self.head.targets())

        current = read_current_positions(self.robot)
        action = {
            f"{joint}.pos": current[joint] + 0.5 * wrapped_error(target, current[joint])
            for joint, target in targets.items()
            if joint in current
        }
        if self.has_base:
            send = not self.wheel_dry_run
            action["x.vel"] = x_vel if send else 0.0
            action["theta.vel"] = theta_vel if send else 0.0
        if action:
            self.robot.send_action(action)

        self._update_leds()
        if self.tick % 25 == 0:
            print(self.debug_line())

    def _head_str(self):
        if self.head is None:
            return ""
        return f" | head tilt={self.head.target[HEAD_TILT_MOTOR]:+.1f} pan={self.head.target[HEAD_PAN_MOTOR]:+.1f}"

    def debug_line(self):
        if self.mode != MODE_WHEEL:
            return "[dbg ARM] " + "  |  ".join(arm.debug_str() for arm in self.arms) + self._head_str()
        d = self.drive
        x_vel, theta_vel = self.last_cmd
        if d.brake:
            state = "BRAKE"
        elif d.neutral is None:
            state = "settling"
        else:
            state = "drive"
        line = (
            f"[dbg WHEEL] {state}: tilt fwd={d.tilt_fwd:+5.1f} turn={d.tilt_turn:+5.1f} "
            f"(total {d.tilt_total:4.1f}) deg -> x.vel={x_vel:+.3f} m/s theta.vel={theta_vel:+5.1f} deg/s"
        )
        if self.wheel_dry_run:
            line += " [DRY-RUN, sent 0]"
        return line + self._head_str()

    def print_banner(self):
        print("Running. ARM mode -- per Joy-Con: stick left/right=pan, stick up/down=lift, X/B (L: up/down)=elbow,")
        print("  gyro=wrist, hold ZR (L: ZL)=move gripper (each new press flips open<->close,")
        print(f"  first press = {'close' if GRIPPER_START_OPENING else 'open'}), plus (L: minus)=wrist recenter.")
        if self.wheel_mode_available:
            print("  Any mode: hold right R + right stick = center camera (right arm ignores the stick meanwhile).")
            print("  L3 (left stick click) = toggle ARM <-> WHEEL (starts in ARM).")
            print("  WHEEL mode: left tilt fwd/back = drive, left/right = turn, ZL = brake toggle,")
            print("  right X/B/Y/A = camera up/down/left/right, right plus = camera center.")
            if self.wheel_dry_run:
                print("  --wheel-dry-run: wheel commands are computed and printed, but 0 is sent.")
        elif self.has_base:
            print("  WHEEL mode unavailable: it needs both Joy-Cons (--arms left,right).")
        print("Ctrl+C to stop.")


def make_joycon(side, calibration, prefix):
    """팔 하나를 위한 Joy-Con 컨트롤러를 만든다. nudge 한계는 그 팔 자신의
    캘리브레이션 항목 (접두사 붙은 관절 이름)에서 읽는다."""
    limits = {
        j: joint_half_range_deg(calibration, f"{prefix}{j}")
        for j in ("shoulder_pan", "shoulder_lift", "elbow_flex")
    }
    if WRIST_LIMIT_FROM_CALIBRATION:
        for j in ("wrist_flex", "wrist_roll"):
            limits[j] = joint_half_range_deg(calibration, f"{prefix}{j}")
    else:
        limits["wrist_flex"] = WRIST_FLEX_LIMIT_DEG
        limits["wrist_roll"] = WRIST_ROLL_LIMIT_DEG
    print(
        f"[{side}] joint limits (margin={JOINT_LIMIT_MARGIN_DEG:.0f} deg): "
        f"pan=+-{limits['shoulder_pan']:.1f} lift=+-{limits['shoulder_lift']:.1f} "
        f"elbow=+-{limits['elbow_flex']:.1f} wrist_flex=+-{limits['wrist_flex']:.1f} "
        f"wrist_roll=+-{limits['wrist_roll']:.1f}"
        + ("" if WRIST_LIMIT_FROM_CALIBRATION else " (wrist: fixed, WRIST_LIMIT_FROM_CALIBRATION=False)")
    )
    print(f"Connecting Joy-Con ({side}) -- hold it level, ~2s...")
    return JointNudgeJoyconRobotics(
        side,
        motor1_limit_deg=limits["shoulder_pan"],
        motor2_limit_deg=limits["shoulder_lift"],
        motor3_limit_deg=limits["elbow_flex"],
        wrist_flex_limit_deg=limits["wrist_flex"],
        wrist_roll_limit_deg=limits["wrist_roll"],
    )


def connect_xlerobot(args):
    """XLeRobot 0.4.0 (2바퀴 차동 베이스, SO101 팔 2개, 헤드).
    --arms로 선택한 팔들에 대해 (robot, {side: prefix})를 반환."""
    try:
        from lerobot.robots.xlerobot_2wheels import XLerobot2Wheels, XLerobot2WheelsConfig
    except ModuleNotFoundError as e:
        if e.name == "zmq":
            raise SystemExit(
                "XLeRobot's xlerobot_2wheels package imports pyzmq (for its host/client). Install it:\n"
                '    pip install "pyzmq>=26.2.1,<28.0.0"'
            ) from e
        if e.name and e.name.startswith("lerobot.robots.xlerobot_2wheels"):
            raise SystemExit(
                "lerobot.robots.xlerobot_2wheels not found. Copy XLeRobot's robot package into lerobot:\n"
                "    cp -r XLeRobot/software/src/robots/xlerobot_2wheels lerobot/src/lerobot/robots/"
            ) from e
        raise

    config = XLerobot2WheelsConfig(
        id=args.robot_id,
        port1=args.port1,
        port2=args.port2,
        use_degrees=True,  # 필수: 기본값 RANGE_M100_100이면 여기 있는 모든 도 단위 상수가 깨진다
    )
    robot = XLerobot2Wheels(config)

    # XLeRobot은 2026-09-18에 왼쪽 바퀴 방향을 고쳤다 (commit 9f664e87,
    # "invert left wheel command"). 그 이전 복사본이면 x.vel이 제자리 회전이
    # 되고 theta.vel이 직진이 된다 -- 그리고 --wheel-dry-run으로는 이걸 알 수
    # 없다. 순수 계산이라 하드웨어는 건드리지 않는다.
    raw = robot._body_to_wheel_raw(0.1, 0.0)
    if not (raw["base_left_wheel"] < 0 < raw["base_right_wheel"]):
        raise SystemExit(
            "Your lerobot/src/lerobot/robots/xlerobot_2wheels is older than XLeRobot's 2026-09-18 left-wheel fix\n"
            "(forward would spin the base in place). Update and re-copy it:\n"
            "    git -C ~/repos/joycon/XLeRobot pull\n"
            "    cp -r ~/repos/joycon/XLeRobot/software/src/robots/xlerobot_2wheels ~/repos/joycon/lerobot/src/lerobot/robots/"
        )

    # 이 id의 캘리브레이션 파일이 없으면, connect()는 ENTER/'c' 프롬프트를
    # 건너뛰고 곧바로 전체 수동 캘리브레이션에 들어가서 모든 팔/헤드 모터의
    # homing offset을 새로 쓴다. 의도했을 때만 그렇게 할 것.
    if not robot.calibration_fpath.is_file() and not args.calibrate:
        found = sorted(p.stem for p in robot.calibration_dir.glob("*.json"))
        raise SystemExit(
            f"No calibration file {robot.calibration_fpath}.\n"
            f"Existing ids in {robot.calibration_dir}: {found or 'none'}\n"
            "Re-run with --robot-id <one of those>, or add --calibrate to run XLeRobot's full calibration now."
        )

    # XLeRobot의 calibrate()/configure()는 레지스터를 lerobot 기본값
    # num_retry=0으로 쓴다. 그래서 상태 패킷 하나만 놓쳐도 수동 캘리브레이션
    # 전체가 저장 전에 중단된다 (2026-09-30 실제 발생: bus2에서 "Failed to
    # write 'Max_Position_Limit' on id_=2 ... There is no status packet!",
    # 직후 두 버스 합쳐 ping 800회는 전부 성공). 쓰기는 멱등이라 재시도한다.
    for bus in (robot.bus1, robot.bus2):
        _write = bus.write

        def write_with_retry(data_name, motor, value, *, normalize=True, num_retry=3, _write=_write):
            return _write(data_name, motor, value, normalize=normalize, num_retry=max(num_retry, 3))

        bus.write = write_with_retry

    # 프롬프트: ENTER = 파일에서 캘리브레이션 복원 / 'c' = 재캘리브레이션
    # (파일이 없고 --calibrate를 줬으면 곧바로 캘리브레이션)
    robot.connect()
    robot.stop_base()  # 이전 크래시에서 남아 있을 수 있는 Goal_Velocity 제거
    prefixes = {side: f"{side}_arm_" for side in args.arms}
    return robot, prefixes


def connect_so101(args):
    """원래의 단일 팔 벤치 리그: SO101Follower 하나, Joy-Con 하나."""
    from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
    from lerobot.robots.so_follower.so_follower import SO101Follower

    robot = SO101Follower(SO101FollowerConfig(port=args.port))
    robot.connect()
    if input("Recalibrate robot? (y/n): ").strip().lower() in ("y", "yes"):
        robot.calibrate()
    return robot, {args.side: ""}


def list_serial_ports():
    """pyserial에서 보이는 시리얼 포트 (lerobot[feetech]와 함께 설치됨)."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [f"{p.device} ({p.description})" for p in list_ports.comports()]


def check_joycons():
    """joyconrobotics와 같은 방식으로 Joy-Con을 나열하고, JoyCon.__init__ /
    JoyconRobotics.__init__에서 하는 것과 같은 시리얼 번호 검사를 적용한다.
    왼쪽 하나와 오른쪽 하나가 받아들여질 수 있으면 True를 반환."""
    import hid
    from joyconrobotics.constants import (
        JOYCON_L_PRODUCT_ID,
        JOYCON_R_PRODUCT_ID,
        JOYCON_SERIAL_HEAD,
        JOYCON_SERIAL_SUPPORT,
        JOYCON_VENDOR_ID,
    )

    sides = {JOYCON_L_PRODUCT_ID: "left", JOYCON_R_PRODUCT_ID: "right"}
    accepted = set()
    found = [d for d in hid.enumerate(0, 0) if d.get("vendor_id") == JOYCON_VENDOR_ID and d.get("product_id") in sides]
    if not found:
        print("No Joy-Con found. Pair them first (Windows: Settings > Bluetooth; Linux: bluetoothctl + joycond).")
    for d in found:
        side = sides[d["product_id"]]
        raw_serial = d.get("serial") or d.get("serial_number")
        serial = mac_with_colons(raw_serial)  # 원격조작 경로에서 적용하는 것과 같은 정규화
        if serial != raw_serial:
            print(f"  ({side}: serial {raw_serial!r} reported without colons -> using {serial!r})")
        product = d.get("product_string")
        problems = []
        if not product:
            problems.append("empty product_string (joyconrobotics skips such devices)")
        if not serial:
            problems.append("no serial number")
        else:
            standard = serial[:12] in JOYCON_SERIAL_HEAD
            mac = len(serial) == 17 and serial.count(":") == 5
            if not (standard or mac):
                problems.append("serial is neither 9c:54:00:b0:/e0:... nor a colon-separated MAC")
            elif serial[:6] != JOYCON_SERIAL_SUPPORT and not mac:
                problems.append("serial rejected by JoyconRobotics ('There is no joycon for robotics')")
        verdict = "OK" if not problems else "NOT USABLE: " + "; ".join(problems)
        print(f"  {side:5s} product={product!r} serial={serial!r} -> {verdict}")
        if not problems:
            accepted.add(side)
    both = accepted == {"left", "right"}
    print("Result:", "both Joy-Cons usable." if both else f"usable: {sorted(accepted) or 'none'} -- both left and right are needed for WHEEL mode.")
    return both


def parse_args():
    parser = argparse.ArgumentParser(description="Joy-Con joint-nudge teleop for XLeRobot 0.4.0 (or a single SO101)")
    parser.add_argument("--robot", choices=["xlerobot", "so101"], default="xlerobot")
    # xlerobot
    parser.add_argument("--robot-id", default="my_xlerobot_2wheels", help="XLerobot2WheelsConfig.id (calibration file name)")
    parser.add_argument("--port1", default=None, help="bus1: left arm + head (Linux default /dev/ttyACM0; Windows: COMx, required)")
    parser.add_argument("--port2", default=None, help="bus2: right arm + wheels (Linux default /dev/ttyACM1; Windows: COMx, required)")
    parser.add_argument(
        "--arms", default="left,right",
        help="which arms to drive, comma-separated subset of left,right (each needs its own Joy-Con of that side)",
    )
    parser.add_argument(
        "--wheel-dry-run", action="store_true",
        help="(xlerobot) compute and print wheel commands but always send 0 -- for checking tilt directions",
    )
    parser.add_argument(
        "--calibrate", action="store_true",
        help="(xlerobot) allow a full calibration when no calibration file exists for --robot-id",
    )
    parser.add_argument(
        "--check-joycons", action="store_true",
        help="only list paired Joy-Cons and check them against joyconrobotics' serial rules, then exit",
    )
    # so101
    parser.add_argument("--port", default=None, help="(--robot so101) follower USB port (Linux default /dev/ttyACM0; Windows: COMx, required)")
    parser.add_argument("--side", choices=["right", "left"], default="right", help="(--robot so101) which Joy-Con")
    args = parser.parse_args()
    if args.check_joycons:
        return args
    needed = ("port1", "port2") if args.robot == "xlerobot" else ("port",)
    missing = [name for name in needed if getattr(args, name) is None]
    if missing:
        if sys.platform.startswith("linux"):
            linux_defaults = {"port1": "/dev/ttyACM0", "port2": "/dev/ttyACM1", "port": "/dev/ttyACM0"}
            for name in missing:
                setattr(args, name, linux_defaults[name])
        else:
            parser.error(
                f"on {sys.platform} there is no default serial port -- pass "
                + " ".join(f"--{name} <COMx>" for name in missing)
                + f". Serial ports found: {list_serial_ports() or 'none'}"
            )
    args.arms = [a.strip() for a in args.arms.split(",") if a.strip()]
    bad = [a for a in args.arms if a not in ("left", "right")]
    if bad or not args.arms:
        parser.error(f"--arms must be a non-empty subset of left,right (got {args.arms})")
    return args


class StopRequest:
    """제어 루프 도중의 Ctrl+C / SIGTERM / SIGHUP / Ctrl+Z는 플래그만 세운다;
    루프는 틱 경계에서 끝난다. 버스 읽기 한가운데서 KeyboardInterrupt가
    나면 시리얼 포트가 사용 중(busy)으로 표시된 채 남아서, 이어지는
    stop_base()가 실패하고 바퀴가 마지막 속도를 유지하게 된다. (Ctrl+Z는
    그냥 두면 바퀴에 명령이 걸린 채로 프로세스를 멈춰버린다.)"""

    SIGNALS = ("SIGINT", "SIGTERM", "SIGHUP", "SIGTSTP")

    def __init__(self):
        self.requested = False
        self._previous = {}

    def _handler(self, signum, frame):
        if not self.requested:
            print("\nStop requested -- finishing this tick, then stopping the base.")
        self.requested = True

    def install(self):
        import signal

        for name in self.SIGNALS:
            sig = getattr(signal, name, None)
            if sig is not None:
                self._previous[sig] = signal.signal(sig, self._handler)

    def restore(self):
        import signal

        for sig, previous in self._previous.items():
            signal.signal(sig, previous)
        self._previous.clear()


def shutdown(robot, arms):
    """시리얼 포트가 어떤 상태로 남아 있든 베이스부터 먼저 멈추고,
    그다음 전부 연결 해제한다. 절대 예외를 던지지 않는다."""
    buses = [getattr(robot, name, None) for name in ("bus1", "bus2", "bus")]
    for bus in buses:
        if bus is None:
            continue
        try:
            # lerobot 자체의 MotorsBus.disconnect()가 토크를 끄기 전에 하는 것:
            # 중단된 트랜잭션이 남긴 busy 플래그를 지운다.
            bus.port_handler.clearPort()
            bus.port_handler.is_using = False
        except BaseException:
            traceback.print_exc()
    if hasattr(robot, "stop_base"):
        for attempt in range(3):
            try:
                robot.stop_base()
                break
            except BaseException:
                traceback.print_exc()
                print(f"[shutdown] stop_base failed (attempt {attempt + 1}/3)")
    try:
        robot.disconnect()
    except BaseException:
        traceback.print_exc()
    # Joy-Con: solve loop만 멈춘다. joyconrobotics의 disconnect()는 리더
    # 스레드가 hid_read()에서 블록된 상태로 HID 핸들을 닫는데, Windows에서
    # 세그폴트가 났다 (2026-09-30 확인); 핸들은 프로세스 종료 시 닫힌다.
    for arm in arms:
        arm.jc.running = False
    print("Disconnected.")


def main():
    args = parse_args()
    if args.check_joycons:
        sys.exit(0 if check_joycons() else 1)

    if args.robot == "xlerobot":
        robot, prefixes = connect_xlerobot(args)
    else:
        robot, prefixes = connect_so101(args)

    arms = []
    stop = StopRequest()
    exit_code = 0
    try:
        for side, prefix in prefixes.items():
            arms.append(ArmBinding(side, prefix, make_joycon(side, robot.calibration, prefix)))

        move_to_zero_position(robot, prefixes=tuple(prefixes.values()))
        current = read_current_positions(robot)
        for arm in arms:
            arm.capture_base(current)

        teleop = Teleop(
            robot, arms, has_base=args.robot == "xlerobot", wheel_dry_run=args.wheel_dry_run, current=current
        )
        control_period = 1.0 / 50
        teleop.print_banner()
        stop.install()
        while not stop.requested:
            teleop.step(time.monotonic())
            time.sleep(control_period)
        print("Stopped by user.")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        stop.restore()
        shutdown(robot, arms)
    # 여기까지 오면 로봇 정리는 끝났다. joyconrobotics의 리더 스레드(daemon)는
    # hid read에서 계속 블록돼 있어서, 인터프리터 정상 종료 중에 stderr 락과
    # 부딪혀 "Fatal Python error: _enter_buffered_busy"를 낸다 (2026-09-30,
    # Windows). 출력을 비우고 finalization 없이 바로 끝낸다.
    if arms:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
