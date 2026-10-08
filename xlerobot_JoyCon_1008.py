#!/usr/bin/env python3
"""XLeRobot Joy-Con 동작 모델: 버튼 없이 스틱과 Joy-Con 방향으로 팔 모터 1·2·3을 움직인다.

2026-10-08 (xlerobot_JoyCon_1008.py). 실행하는 텔레옵이 아니라 모듈이다. 로봇이나
Joy-Con에 연결하지 않고 (import는 math와 dataclasses뿐), 입력값을 받아 관절 목표를
계산만 한다. 당분간 로봇과 Joy-Con으로 테스트할 수 없어서 (사용자, 2026-10-08) PoC를
이 파일 안에서 완성하고, 나중에 Final_MMDD 텔레옵과 시뮬레이터가 import해서 쓴다.

사용자 목표 (2026-10-08): "리더-팔로워처럼 wrist_roll, wrist_flex, gripper를 제외한
1, 2, 3번 모터를 가능한 한 조이스틱만으로." 스틱은 2축이고 모터는 3개라서 세 번째를
Joy-Con 본체를 좌/우로 돌리는 방향 (yaw)에 맡긴다. 손목 자이로는 앞뒤로 숙이기
(gy -> wrist_flex)와 긴 축 비틀기 (gx -> wrist_roll)에 쓰이고, 좌/우로 돌리기는 아직 아무 데도
안 쓴다. (공식 XLeRobot 예제 중 yaw를 pan에 쓰는 것은 없다. 7_ 예제는 yaw를 안 쓰고,
6_ 예제는 스틱 앞 방향에 cos(yaw)로만 쓴다. Box2AI의 lerobot_joycon_gpos_real.py와
lerobot-joycon2는 pan = yaw지만 높이는 버튼이다. 그대로 쓸 만한 예제가 없어서 새로 짰다.)

동작 (Joy-Con마다 자기 팔, 이 모델이 맡는 부분만):
    Joy-Con을 좌/우로 돌림 -> shoulder_pan. 위치 제어: Joy-Con 긴 축이 가리키는 방향이
                             돈 각도 x YAW_TO_PAN_GAIN만큼 팔이 돈다 (손이 가리키는 쪽을
                             팔이 따라감).
    스틱 위/아래           -> 그리퍼 앞/뒤 (팔 평면 안에서 수평, IK). newcontrol과 같음.
    스틱 좌/우             -> 그리퍼 위/아래 (수직, IK). newcontrol의 X/B 대신.
  Final_1007_newcontrol과 비교:
      입력                  newcontrol              1008 동작 모델
      스틱 좌/우            shoulder_pan (속도)     그리퍼 위/아래
      스틱 상/하            그리퍼 앞/뒤            같음
      X/B (L: 위/아래)      그리퍼 위/아래          필요 없음
      Joy-Con 좌/우 회전    안 씀                   shoulder_pan (위치)
  손목 (자이로), 그리퍼 (ZR/ZL), plus/minus, home/capture, L3, R+스틱 카메라, WHEEL 모드는
  이 모델 밖이다. 텔레옵 쪽에서 newcontrol 그대로 쓴다. 스틱 축 배치는 STICK_AXES로 바꾼다.

yaw 구하기 (YawEstimator):
  - Joy-Con의 자세를 계속 추적한다: 중력축 (가속도계 방향)과 보정 때 정한 수평 기준 방향,
    둘 다 Joy-Con 좌표로. 자이로로 같이 돌리고, 중력축만 가속도계로 천천히 바로잡는다.
    yaw는 적분하지 않고 매번 이 자세에서 바로 계산한다:
      * 평소 (긴 축이 수평에서 YAW_HEADING_FULL_DEG 안): Joy-Con 긴 축 (로컬 X, wrist_roll이 쓰는
        gx의 축)이 수평면에서 가리키는 방향. 긴 축을 도는 비틀기 (wrist_roll 입력)는 pan을
        움직이지 않는다.
      * 긴 축이 많이 서면 (YAW_HEADING_NONE_DEG 이상) 방향이 정해지지 않으므로, 보정 때 자세에서
        수직축을 돈 각도 (swing-twist의 twist)를 쓴다. 그 사이는 두 각도를 섞는다. 여기서는 긴
        축 비틀기가 다시 pan에 섞인다. 그래서 Joy-Con을 세로로 세워 쥐는 파지는 맞지 않는다
        (2026-09-30 실측 파지는 눕혀 쥠: 가속도계 기준 중력이 로컬 -Z).
      * 둘 다 자세만으로 정해지는 값이라, 손목을 어떻게 움직였든 같은 자세로 돌아오면 pan도
        같은 값으로 돌아온다 (경로와 상관없음). 좌/우로 돌리는 것은 어느 기울기에서도 그대로 센다.
      * 손목을 앞뒤로 숙일 때는, Joy-Con을 옆으로 기울여 쥐고 있으면 코끝이 실제로 옆으로
        움직이므로 그만큼 pan이 움직인다 (평평하게 쥘수록 작다).
      * Joy-Con 코끝을 수직 너머까지 넘기거나 (뒤집기) 보정 때와 반대로 엎으면 각도가 튈 수 있다.
  - 시작할 때 자이로 바이어스를 한 번 재서 뺀다 (사용자 결정, 2026-10-08).
    start_calibration() 뒤 Joy-Con을 YAW_CALIB_S 동안 책상에 내려놓는다 (손에 든 채로는 안 됨:
    손을 천천히 돌리고 있으면 그 회전이 바이어스로 들어가서 pan이 계속 흐른다. 이것은
    정지 측정으로는 알아낼 수 없다). 자이로나 가속도가 흔들리면 처음부터 다시 재고,
    YAW_CALIB_MAX_TRIES번 연속 실패하면 가장 나은 측정을 쓰고 calib_ok = False로 알린다.
  - 리포트마다 IMU 샘플 3개를 각각 5 ms로 적분한다 (dekuNukem imu_sensor_notes).
    newcontrol 손목은 첫 샘플 하나를 고정 0.01초로 적분한다 (CHANGELOG 1006은 이것을 빠른
    회전 때 손목이 덜 따라가는 원인 후보로 적었다).
  - joyconrobotics의 gyro_in_rad는 실제 rad/s의 절반이라 두 배로 고쳐 쓴다
    (JOYCONROBOTICS_GYRO_RAD_FIX 참고).
  - 아주 느린 회전은 줄인다 (YAW_TIGHTEN_RADPS). 남은 바이어스 드리프트를 조금 누른다.
  - 자력계가 없어서 절대 방향은 모른다. 시간이 지나거나 온도가 바뀌면 조금씩 어긋날 수
    있다. rezero_yaw()로 팔을 움직이지 않고 지금 손 방향을 지금 팔 방향으로 다시 맞춘다.

팔 (StickYawArm):
  - IK, 한계에서 움직임을 버리고 멈추는 동작, IK_X_MIN_M은 newcontrol 그대로다.
  - 시작 (set_home) 직후에는 pan이 yaw를 따라가지 않는다 (engaged = False). Joy-Con을 책상에서
    집어 드는 동작이 pan으로 가면 안 되기 때문이다. yaw가 있을 때 스틱을 밀거나
    (ENGAGE_ON_STICK) engage()를 부르면, 그 순간의 손 방향을 지금 팔 방향으로 잡고 따라가기
    시작한다 (yaw가 없는 동안의 engage()는 yaw가 들어오는 순간으로 미뤄진다).
    다시 내려놓을 때 (예: 바이어스를 다시 잴 때)는 disengage(). 그 뒤에는 스틱이 한 번
    가운데로 돌아왔다가 다시 밀려야 따라가기 시작한다.
  - pan이 한계에 닿으면 멈춘다. 손을 더 돌리면 기준점을 같이 밀어서 (PAN_DRAG_AT_LIMIT),
    손을 되돌리는 순간 팔이 바로 돌아온다.
  - yaw가 없을 때 (보정 전/중, Joy-Con 끊김, WHEEL 모드) pan 목표는 그 자리에 선다. yaw가
    돌아오면 지금 pan에서 이어간다. 그사이 손이 돌아가 있어도 팔이 튀지 않는다.
  - 출력 목표에 속도 제한 (PAN_MAX_DEG_S, LIFT_ELBOW_MAX_DEG_S)을 둔다. home, 시작 자세 보정,
    손을 휙 돌릴 때 목표가 한 번에 튀지 않는다 (newcontrol의 home은 목표가 한 번에 바뀜).
    입력이 멈춰도 출력은 마지막 목표까지 마저 간 뒤 선다 (pan은 최대 약 1.8초).
  - NaN/inf 입력은 무시한다 (yaw는 None과 같게, 스틱은 0, dt는 0).

텔레옵에 붙이는 법 (예정, 아직 안 붙였다):
    import xlerobot_JoyCon_1008 as motion   # 별표 import 금지: 이름이 newcontrol과 겹친다
        # 팔 조작 숫자 (STICK_DEADZONE, STICK_REACH_SIGN 등)는 이 모듈의 것만 쓰인다.
        # Final 쪽에 같은 이름의 상수를 남겨 두고 고치면 팔에는 반영되지 않는다.
    yaw = motion.YawEstimator()
    jc.gyro.register_update_hook(hook)
        # hook(dev) 안에서 yaw.add_joyconrobotics_report(dev, time.perf_counter()).
        # 예외는 hook 안에서 반드시 잡는다 (훅에서 예외가 나면 그 HID 리더 스레드가 죽는다).
        # joyconrobotics는 Joy-Con을 세 번 연다 (joycon/gyro/button). 셋 다 같은 리포트를 받으므로
        # 훅은 한 핸들 (IMU 보정값을 수리한 jc.gyro)에만 단다.
        # joyconrobotics가 자체 바이어스를 재는 중 (dev.is_calibrating)인 리포트는 버리고, 그게
        # 끝나면 이 모듈이 바이어스를 다시 잰다 (오프셋이 바뀌므로). 따로 맞출 필요는 없다.
    JointNudgeJoyconRobotics 생성자가 끝난 뒤, Joy-Con을 책상에 내려놓으라고 안내하고
    yaw.start_calibration(). yaw.ready를 기다리되 시간 제한 (YAW_CALIB_S x
    (YAW_CALIB_MAX_TRIES + 1) + 2초쯤)을 둔다. 리포트가 안 오거나 값이 NaN이면 (bad_reports)
    끝나지 않는다. calib_ok가 False면 다시 내려놓으라고 하고 start_calibration()을 다시 부른다.
    그다음 "Joy-Con을 들고 정면을 가리킨 뒤 스틱을 밀면 시작"이라고 안내한다.
    arm = motion.StickYawArm(side, pan_limits, lift_limits, elbow_limits)
        # (최소, 최대) 도. 캘리브레이션 한계에서 여유를 뺀 값, newcontrol 기준으로는
        # (-joint_half_range_deg, +joint_half_range_deg)
    arm.set_home(pan, lift, elbow)          # 영점 이동 뒤 실제 관절값. 이 전에는 step()이 None
    매 틱 (dt는 time.perf_counter() 차이. Windows Python 3.12의 time.monotonic()은 15.6 ms 단위라
    틱의 3분의 1쯤에서 dt가 0이 된다):
        ARM 모드, 살아있는 데이터:        cmd = arm.step(dt, sx, sy, yaw.yaw_deg)
        그중 스틱만 막을 때 (stick_live 전, STICK_CLICK_HOLDOFF_S 안):
                                          cmd = arm.step(dt, None, None, yaw.yaw_deg)
        팔을 멈출 때 (WHEEL 모드, Joy-Con 끊김, R+스틱 카메라 중): arm.step(dt, None, None, None)
            # 목표는 멈추고 (출력은 남은 거리를 마저 간다), 다음에 yaw가 들어오면 지금 pan에서
            # 이어간다. 이 호출을 빼먹으면 WHEEL 모드에서 Joy-Con을 돌린 만큼 ARM으로 돌아오는
            # 순간 pan이 따라 돈다.
        jc.mode가 None일 때 (set_home 전)는 부르지 않는다.
    로봇 목표: shoulder_pan/lift/elbow_flex = cmd.pan_deg / lift_deg / elbow_deg.
        wrist_flex = clamp(손목 자이로 값 + (cmd.wrist_flex_comp_deg if KEEP_GRIPPER_ANGLE else 0),
                           +-wrist_flex_limit_deg)
            # newcontrol의 ArmBinding.wrist_flex_target과 같은 모양. 다만 보정값의 기준은 실제
            # 시작 자세이고, 속도 제한을 거친 출력으로 계산한다.
    손목 자이로: newcontrol의 손목 적분기에 원시 자이로 대신 yaw.split_heading(g)[1]을 넣는다.
        그러지 않으면 pan을 위해 Joy-Con을 돌릴 때마다 손목도 같이 돈다 (2026-09-30 파지에서
        돌린 각도의 약 19%가 wrist_roll, 6%가 wrist_flex로 샌다, 리뷰 계산).
        또 arm.engage_count가 바뀔 때 (pan이 따라가기 시작할 때) 손목 적분기를 0으로 맞추고,
        cmd.engaged가 False인 동안은 손목 적분도 멈춘다. 그래야 Joy-Con을 내려놓고 집어 드는
        동작이 손목으로 들어가지 않는다.
    home (L: capture)를 남겨 두면 arm.go_home(). 다시 잴 때는 arm.disengage() 뒤
        yaw.start_calibration().
  스레드: add_joyconrobotics_report는 HID 리더 스레드에서, step은 joyconrobotics solve_loop
  스레드에서 불린다. 메인 스레드는 arm.command (step마다 새로 만드는 불변 객체)를 한 번
  읽어서 쓴다. go_home / rezero_yaw / engage는 요청만 남기고 다음 step()이 처리하므로 다른
  스레드에서 불러도 된다. disengage는 바로 적용된다. set_home은 step()을 돌리기 전에 부른다.

검증 상태 (2026-10-08): 테스트 파일은 아직 없다 (사용자 요청으로 보류). 합성 IMU 데이터로 임시
확인과 리뷰 두 번 (리뷰어 4명 + 검증 4명, 고친 뒤 리뷰어 2명 + 검증 1명)만 했다. 시뮬레이터와
실기는 아직이다. 부호 (YAW_SIGN, YAW_PAN_SIGN, STICK_REACH_SIGN, STICK_HEIGHT_SIGN)와 모든
숫자는 미검증/미튜닝이다.
"""

import math
from dataclasses import dataclass

__all__ = ["YawEstimator", "StickYawArm", "ArmCommand"]

# ---------------------------------------------------------------------------
# Joy-Con IMU -> yaw
# ---------------------------------------------------------------------------
# 0x30 입력 리포트 하나에 IMU 샘플 3개가 5 ms 간격으로 들어 있다. 리포트는 약 15 ms마다 온다
# (dekuNukem imu_sensor_notes: "1st sample 0ms, 2nd 5ms, 3rd 10ms"; 순서상 첫 샘플이
# 가장 오래된 것으로 본다).
IMU_SAMPLE_DT_S = 0.005

# joyconrobotics의 gyro_in_rad는 실제 rad/s의 절반이다. 같은 파일의 gyro_in_rot는
# 회전수/초 (0.0001694/LSB)인데, gyro_in_rad는 거기에 2*pi가 아니라 pi를 곱한다
# (joycon-robotics/joyconrobotics/wrappers.py, 2026-10-08 코드로 확인). 그래서 그 값을 넣을 때
# 이 배수를 곱한다 (add_joyconrobotics_report가 알아서 한다). newcontrol의 손목 게인은
# 이 절반 값에 맞춰 실기 튜닝됐으므로 손목 쪽은 건드리지 않는다.
JOYCONROBOTICS_GYRO_RAD_FIX = 2.0
# 자이로 감도에도 두 가지 값이 있다: 0.061 dps/LSB (4000/65535, joyconrobotics가 씀)와
# 0.070 dps/LSB (4588/65535, LSM6DS3 데이터시트 감도, dekuNukem의 "saturation-free" 공식).
# 어느 쪽이 맞는지 미확인이라 1.0 (= joyconrobotics 그대로). 실기에서 Joy-Con을 90도 돌렸는데
# yaw가 약 78도로 나오면 4588 / 4000 (= 1.147)으로 바꾼다.
GYRO_SCALE_FIX = 1.0

# 시작 바이어스 측정. Joy-Con을 책상에 내려놓은 YAW_CALIB_S 동안의 자이로 평균을 바이어스로
# 쓴다. joyconrobotics도 생성할 때 2초 바이어스를 빼지만 (GyroTrackingJoyCon.calibrate), 그때
# 손에 들고 있었으면 남은 바이어스가 크다. 이 측정은 그 위에 남은 것을 뺀다.
YAW_CALIB_S = 2.0
YAW_CALIB_SAMPLES = round(YAW_CALIB_S / IMU_SAMPLE_DT_S)
# 측정 중 자이로 축별 표준편차가 이보다 크면 "움직였다"로 보고 처음부터 다시 잰다. 정지
# 상태의 노이즈는 약 0.0013-0.0015 rad/s rms로 추정했다 (LSM6DS3 데이터시트 7 mdps/sqrt(Hz)와
# 양자화로 계산한 값, 실측 아님). 미튜닝.
YAW_CALIB_MAX_STD_RADPS = 0.01
# 가속도 축별 표준편차가 이보다 크면 역시 다시 잰다. 책상 위에서는 약 1 mg (LSM6DS3 약
# 0.1 mg/sqrt(Hz)로 계산), 손에 들면 0.5도만 흔들려도 수평 축이 약 9 mg 움직인다. 손에 든 채
# 잰 측정을 거르는 용도다 (자이로 std만으로는 손떨림이 작으면 통과한다). 미튜닝.
YAW_CALIB_MAX_ACCEL_STD_G = 0.005
# 측정 중 평균 가속도 크기가 이 범위 밖이면 (흔들림, 낙하, IMU 보정값 이상) 그 측정은 실패다.
YAW_CALIB_ACCEL_MIN_G = 0.8
YAW_CALIB_ACCEL_MAX_G = 1.2
# 이만큼 연속으로 실패하면 그중 가장 나은 측정을 쓰고 calib_ok = False로 알린다
# (끝없이 기다리지 않게). 2초 x 5 = 약 10초.
YAW_CALIB_MAX_TRIES = 5

# 중력축 (self.up, Joy-Con 좌표의 단위벡터, 가속도계가 가리키는 방향). 자이로로 같이 돌리고
# (지연 없음), 가속도계로 천천히 바로잡는다 (드리프트 없음). 팔을 돌리는 동안의 원심/접선
# 가속도는 중력과 거의 직각이라 크기 검사 (0.9-1.1 g)로는 다 걸러지지 않는다. 그래서
#   - Joy-Con이 천천히 돌 때만 (|자이로| < YAW_GRAVITY_MAX_RATE_RADPS) 바로잡는다 (원심 가속도는
#     회전 속도의 제곱이라 여기서 대부분 걸러진다),
#   - 가속도 방향이 지금 중력축에서 YAW_GRAVITY_MAX_ANGLE_DEG 안일 때만 바로잡는다 (움직임이
#     시작하고 멈추는 순간의 접선 가속도는 회전이 느릴 때 크므로 위 조건으로는 안 걸러진다).
#     YAW_GRAVITY_RELAX_S 동안 계속 밖이면 중력축이 틀어진 것으로 보고 다시 받아들인다.
#   - 시정수도 길게 둔다.
# (리뷰 계산: 0.5초 + 크기 검사만으로는 좌우 회전과 손목 굽힘을 번갈아 5번 하면 yaw가
# 2-10도 쌓였다.)
YAW_GRAVITY_TAU_S = 2.0  # 미튜닝
YAW_GRAVITY_ACCEL_MIN_G = 0.9
YAW_GRAVITY_ACCEL_MAX_G = 1.1
YAW_GRAVITY_MAX_RATE_RADPS = 0.5  # 약 29도/초. 미튜닝
YAW_GRAVITY_MAX_ANGLE_DEG = 2.5  # 미튜닝
YAW_GRAVITY_RELAX_S = 2.0  # 미튜닝
# 자이로로 중력축을 돌리려면 가속도계와 자이로가 같은 오른손 좌표계여야 한다. Joy-Con은
# 한 칩 (LSM6DS3)이고, joyconrobotics는 왼쪽 Joy-Con의 Y/Z를 가속도와 자이로 둘 다 뒤집으므로
# 맞을 것으로 본다 (미검증). 확인: Joy-Con을 평평하게 들고 90도 돌린 것과, 45도쯤 앞으로 숙인
# 직후 바로 90도 돌린 것이 둘 다 약 90도로 나와야 한다 (숙이고 몇 초 기다리면 가속도 보정이
# 따라잡아서 차이가 안 보인다). 숙인 쪽만 눈에 띄게 적게 나오면 축 방향이 안 맞는 것이다.
# 확인할 때 YAW_GRAVITY_ACCEL_MIN_G = 2로 가속도 보정을 잠깐 끄면 차이가 훨씬 커진다.
# 그때는 False로: 중력축을 가속도계 로우패스 (YAW_GRAVITY_LPF_TAU_S)로만 구한다. 빠르게
# 기울이는 동안에는 조금 섞인다.
YAW_GRAVITY_GYRO_PROPAGATE = True
YAW_GRAVITY_LPF_TAU_S = 0.1  # YAW_GRAVITY_GYRO_PROPAGATE = False일 때만 쓴다
# 가속도 값을 쓸 수 없을 때의 중력축 (Joy-Con 좌표). 2026-09-30 실측 파지에서 가속도계가
# (-0.20, 0.09, -0.97) g였으므로 -Z.
YAW_FALLBACK_UP = (0.0, 0.0, -1.0)
# 긴 축 (로컬 X)이 수평에서 많이 서면 가리키는 방향이 잘 정해지지 않는다 (중력축 오차가
# 1/cos(기울기)배로 커짐). 그래서 YAW_HEADING_FULL_DEG까지는 긴 축 방향만 쓰고,
# YAW_HEADING_NONE_DEG 이상에서는 보정 때 자세에서 수직축을 돈 각도 (twist)를 쓰고, 그 사이는
# 두 각도를 섞는다. 각도 (자세로 정해지는 값)를 섞으므로 경로와 상관없다. 둘 다 수직축 회전은
# 똑같이 세므로 좌/우 돌리기는 어느 기울기에서도 잃지 않는다. 다만 섞는 구간부터는 긴 축
# 비틀기가 다시 pan에 섞인다. (처음에는 두 회전 속도를 섞어서 적분했는데, 그러면 그 구간을 지나는
# 손목 동작이 pan을 영구히 틀어놨다. 리뷰 계산: 2026-09-30 파지에서 약 48도 숙이면 들어간다.)
YAW_HEADING_FULL_DEG = 60.0
YAW_HEADING_NONE_DEG = 75.0

# 아주 느린 회전은 줄여서 남은 바이어스 드리프트를 누른다 (GyroWiki의 "tightening").
# 자이로의 수직축 성분 |속도| < 이 값이면 속도 x (|속도| / 이 값). 딱 끊는 데드존과 달리
# 부드럽게 이어진다.
# 0.01 rad/s = 0.57 도/초. 남은 바이어스가 0.002 rad/s (분당 6.9도)일 때, 노이즈가 없으면 분당
# 약 1.4도, 책상 위 노이즈 (0.0015 rad/s)에서는 약 2도로 준다. 손에 들고 있을 때는 손떨림이 이
# 값보다 커서 거의 줄지 않는다 (리뷰 계산). 0이면 끈다. 미튜닝.
YAW_TIGHTEN_RADPS = 0.01
# yaw 부호: + = Joy-Con을 오른쪽으로 돌림 (위에서 보아 시계 방향). 가속도계는 정지 상태에서
# 위쪽을 +1 g로 읽고 자이로는 같은 축에서 오른손 법칙이라고 가정하면, 중력축을 도는 회전은
# 왼쪽으로 돌릴 때 +다. 그래서 -1. 미검증: 오른쪽으로 돌렸는데 yaw가 줄어들면 +1로.
# (중력축 기준이라 좌/우 Joy-Con의 축 방향 차이와 상관없이 부호 하나로 된다.)
YAW_SIGN = -1.0
# 리포트 도착 간격이 이보다 길면 late_reports를 하나 센다 (진단용). 도착이 늦기만 하고 리포트는
# 쌓여 있다가 한꺼번에 오면 잃는 것은 없다. 반대로 블루투스에서 리포트가 1-2개 빠지면
# (30-45 ms) 세지 못하고, 그 사이 회전은 잃는다. 0x30 리포트의 timer 바이트 단위가 문서화돼 있지
# 않아서 빠진 리포트를 정확히 세거나 메우지는 못한다.
YAW_REPORT_GAP_S = 0.05

# ---------------------------------------------------------------------------
# 스틱 -> 그리퍼 끝 (IK)
# ---------------------------------------------------------------------------
# 스틱 값은 -1..1 (Joy-Con 스틱 캘리브레이션으로 정규화한 값, newcontrol의 read_stick 그대로).
STICK_DEADZONE = 0.25  # 축별 |값|이 이 이하면 0 (newcontrol과 같음)
STICK_FULL = 0.9  # 이 이상이면 최고 속도 (newcontrol과 같음)
# 램프 모양. 1.0 = 직선 (newcontrol과 같음). 2.0이면 가운데 근처가 느려져 미세 조정이 쉬워지고,
# 끝까지 밀면 같은 최고 속도다. 미튜닝.
STICK_RATE_EXPO = 1.0
# 스틱 축 배치: (가로, 세로)가 각각 무엇을 움직이나. "reach" = 앞/뒤, "height" = 위/아래.
# 기본은 세로 = 앞/뒤 (newcontrol 그대로), 가로 = 위/아래 (pan이 yaw로 옮겨가서 빈 자리).
STICK_AXES = ("height", "reach")
# 스틱을 끝까지 밀었을 때의 속도. newcontrol의 0.0006 m/틱 x 약 96 Hz (sleep(0.01) 루프 실측)
# = 약 5.8 cm/s와 거의 같다. 미튜닝.
IK_REACH_MPS = 0.06
IK_HEIGHT_MPS = 0.06
STICK_REACH_SIGN = 1.0  # 미검증: 스틱 위 = 앞으로 (x +). newcontrol과 같은 상수
# 미검증/취향: 스틱 오른쪽 = 위로 (z +). 좌/우 Joy-Con 따로 (왼손은 오른쪽이 안쪽).
STICK_HEIGHT_SIGN = {"right": 1.0, "left": 1.0}
# step()의 dt가 이보다 크면 잘라 쓴다 (스레드가 멈췄다 돌아왔을 때 한 번에 튀지 않게).
MAX_STEP_DT_S = 0.05

# ---------------------------------------------------------------------------
# yaw -> shoulder_pan, 출력 속도 제한
# ---------------------------------------------------------------------------
# 위치 제어: Joy-Con을 돌린 각도 x 이 배율 = 팔이 도는 각도. 1.0 = 손과 팔이 같은 각도
# (리더-팔로워 느낌). pan 한계 (캘리브레이션 기준 약 +-92 / +-106도)까지 가려면 손을 그만큼
# 돌려야 한다. 손을 덜 돌리고 싶으면 1.5 등으로 올린다. 미튜닝.
YAW_TO_PAN_GAIN = 1.0
YAW_PAN_SIGN = 1.0  # 미검증: Joy-Con 오른쪽 = pan + (newcontrol의 스틱 오른쪽 = pan +와 같은 방향)
# True면 set_home / disengage 뒤 yaw가 있을 때 스틱을 밀면 pan이 yaw를 따라가기 시작한다 (그
# 순간의 손 방향 = 지금 팔 방향). False면 engage()를 불러야만 시작한다.
ENGAGE_ON_STICK = True
# pan이 한계에 닿은 뒤에도 손을 더 돌리면, 기준점을 같이 밀어서 손을 되돌리는 순간 팔이
# 바로 돌아오게 한다 (newcontrol의 "한계에서 멈추고 반대로 당기면 바로 돌아옴"과 같은 생각).
# False면 손이 한계 안으로 다시 들어올 때까지 팔이 한계에 머문다. 단 yaw가 한 번이라도 끊기면
# (WHEEL 모드, 끊김, R+스틱 카메라) 그때 기준점을 한계 위치로 다시 잡으므로, 손과 팔 방향은
# 끊기기 전까지만 일치한다.
PAN_DRAG_AT_LIMIT = True
# ArmCommand.pan_at_limit: pan 목표가 한계에 닿으면 켜지고, 한계에서 이만큼 안쪽으로 들어와야
# 꺼진다 (손떨림에 깜빡이지 않게).
PAN_AT_LIMIT_RELEASE_DEG = 3.0
# 출력 목표의 최대 속도 (도/초). 평소 스틱 이동에서는 팔을 거의 끝까지 뻗은 근처 (작은 이동에
# 관절이 크게 도는 곳)에서만 걸린다. home, 시작 자세 보정, 손을 휙 돌렸을 때 목표가 한 번에
# 튀지 않게 한다. 미튜닝.
PAN_MAX_DEG_S = 120.0
LIFT_ELBOW_MAX_DEG_S = 90.0

# ---------------------------------------------------------------------------
# SO101 IK (Final_1007_newcontrol에서 그대로)
# ---------------------------------------------------------------------------
# 좌표는 팔 평면 (shoulder_pan으로 돌아가는 평면) 안의 (x = 앞으로 뻗은 거리, z = 높이), 미터,
# shoulder_lift 축 기준. 관절 0 자세는 x 16.3 cm, z 11.8 cm.
IK_X_MIN_M = 0.06  # 그리퍼를 어깨 축에서 이보다 뒤로 당기지 않음 (뒤로 넘어가면 몸체/헤드와 부딪힘). 미튜닝
# IK와 so101_fk가 서로 정확히 맞는 관절 범위 (2026-10-07 계산: elbow < 약 -73.8도면
# 팔꿈치가 반대로 꺾인 자세라 IK가 못 만들고, lift > 약 95.7도 / elbow > 90도는 IK 안의
# 한계에 걸린다). 캘리브레이션 한계와 이 범위 중 좁은 쪽으로 자른다.
IK_LIFT_MAX_DEG = 93.0
IK_ELBOW_MIN_DEG = -70.0
IK_ELBOW_MAX_DEG = 88.0

# SO101 팔 치수와 각도 보정 (XLeRobot software/src/model/SO101Robot.py의
# SO101Kinematics에서 그대로).
SO101_L1 = 0.1159  # 위팔 (m)
SO101_L2 = 0.1350  # 아래팔 (m)
SO101_OFF1 = math.atan2(0.028, 0.11257)
SO101_OFF2 = math.atan2(0.0052, 0.1349) + SO101_OFF1

assert 0.0 <= STICK_DEADZONE < STICK_FULL <= 1.0
assert sorted(STICK_AXES) == ["height", "reach"], "STICK_AXES = ('height', 'reach') 순서만 바꿀 수 있다"
assert YAW_TO_PAN_GAIN > 0.0 and STICK_RATE_EXPO > 0.0 and YAW_CALIB_MAX_TRIES >= 1
assert 0.0 < YAW_HEADING_FULL_DEG < YAW_HEADING_NONE_DEG < 90.0

# 긴 축 기울기의 cos^2 (= 1 - ux^2)로 바꾼 경계 (_heading_weight 참고).
_HEADING_FULL_HORIZ2 = math.cos(math.radians(YAW_HEADING_FULL_DEG)) ** 2
_HEADING_NONE_HORIZ2 = math.cos(math.radians(YAW_HEADING_NONE_DEG)) ** 2
_GRAVITY_GATE_COS = math.cos(math.radians(YAW_GRAVITY_MAX_ANGLE_DEG))


def so101_ik(x, z):
    """(x, z) 미터 -> (shoulder_lift, elbow_flex) 도. XLeRobot의
    SO101Kinematics.inverse_kinematics를 그대로 옮긴 것 (닿지 않는 점은 작업
    공간 경계로 당기고, URDF 한계로 자른다)."""
    l1, l2 = SO101_L1, SO101_L2
    r = math.hypot(x, z)
    r_max, r_min = l1 + l2, abs(l1 - l2)
    if r > r_max:
        x, z, r = x * r_max / r, z * r_max / r, r_max
    if 0 < r < r_min:
        x, z, r = x * r_min / r, z * r_min / r, r_min
    cos_t2 = max(-1.0, min(1.0, -(r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2)))
    t2 = math.pi - math.acos(cos_t2)
    t1 = math.atan2(z, x) + math.atan2(l2 * math.sin(t2), l1 + l2 * math.cos(t2))
    joint2 = max(-0.1, min(3.45, t1 + SO101_OFF1))
    joint3 = max(-0.2, min(math.pi, t2 + SO101_OFF2))
    return 90.0 - math.degrees(joint2), math.degrees(joint3) - 90.0


def so101_fk(lift_deg, elbow_deg):
    """so101_ik의 정확한 역함수: (shoulder_lift, elbow_flex) 도 -> (x, z) 미터.
    (공식 파일의 forward_kinematics는 IK와 elbow가 32도 어긋나서 쓰지 않는다.)
    IK_LIFT_MAX_DEG / IK_ELBOW_*_DEG 범위 안에서 so101_ik(so101_fk(a, b)) == (a, b)."""
    t1 = math.radians(90.0 - lift_deg) - SO101_OFF1
    t2 = math.radians(elbow_deg + 90.0) - SO101_OFF2
    return (
        SO101_L1 * math.cos(t1) + SO101_L2 * math.cos(t1 - t2),
        SO101_L1 * math.sin(t1) + SO101_L2 * math.sin(t1 - t2),
    )


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _approach(current, target, max_step):
    """current에서 target 쪽으로 최대 max_step만큼. 닿으면 target 그대로 (같은 값)."""
    d = target - current
    if abs(d) <= max_step:
        return target
    return current + math.copysign(max_step, d)


def _unit(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / n, v[1] / n, v[2] / n)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _wrap(angle):
    """-pi..pi로 감는다."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _finite_samples(samples):
    return all(math.isfinite(c) for s in samples for c in s)


def _heading_weight(up):
    """긴 축 방향 각도의 가중치 (0..1). 긴 축 (로컬 X)이 수평에서 YAW_HEADING_FULL_DEG 안이면 1,
    YAW_HEADING_NONE_DEG 이상 서면 0, 그 사이는 직선. up = 중력축 (Joy-Con 좌표 단위벡터)."""
    horiz2 = 1.0 - up[0] * up[0]  # = cos^2(긴 축의 기울기)
    if horiz2 >= _HEADING_FULL_HORIZ2:
        return 1.0
    if horiz2 <= _HEADING_NONE_HORIZ2:
        return 0.0
    return (horiz2 - _HEADING_NONE_HORIZ2) / (_HEADING_FULL_HORIZ2 - _HEADING_NONE_HORIZ2)


def heading_rate(w, up):
    """각속도 w (Joy-Con 좌표)에서 지금 순간의 yaw 변화율 (YAW_SIGN 적용 전, w와 같은 단위).
    긴 축이 수평에 가까우면 긴 축이 가리키는 수평 방향의 변화율 (ZYX 오일러 yaw 변화율):
    수직축을 도는 회전은 그대로 나오고, 긴 축을 도는 비틀기 (w = (wx, 0, 0))는 0이다. 긴 축이
    많이 서면 수직축 회전 w . up 쪽으로 섞는다 (YawEstimator와 같은 가중치).
    YawEstimator는 이것을 적분하지 않고 yaw를 자세에서 바로 구한다 (경로와 상관없게). 이 함수는
    손목용 split_heading에만 쓴다."""
    wx, wy, wz = w
    ux, uy, uz = up
    vertical = wx * ux + wy * uy + wz * uz  # 수직축을 도는 회전
    alpha = _heading_weight(up)
    if alpha <= 0.0:
        return vertical
    heading = (wy * uy + wz * uz) / (uy * uy + uz * uz)  # = (w . up - wx * ux) / (1 - ux^2)
    return alpha * heading + (1.0 - alpha) * vertical


def stick_to_rate(u):
    """스틱 한 축 (-1..1) -> 속도 비율 (-1..1). |u| <= STICK_DEADZONE -> 0,
    >= STICK_FULL -> +-1, 그 사이는 STICK_RATE_EXPO 모양의 램프. NaN/inf는 0."""
    if not math.isfinite(u):
        return 0.0
    a = abs(u)
    if a <= STICK_DEADZONE:
        return 0.0
    frac = min(1.0, (a - STICK_DEADZONE) / (STICK_FULL - STICK_DEADZONE))
    return math.copysign(frac**STICK_RATE_EXPO, u)


def _twist_about_z(m):
    """회전 행렬 m (3x3, 행의 튜플)을 세 번째 축 (중력축)을 도는 회전 (twist)과 나머지 (swing)로
    나눴을 때의 twist 각도 (rad, -pi..pi). 사원수로 바꿔서 2 * atan2(qz, qw) (Shepperd 방법,
    180도 근처에서도 안정)."""
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = m
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw, qz = 0.25 * s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw, qz = (m21 - m12) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw, qz = (m02 - m20) / s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw, qz = (m10 - m01) / s, 0.25 * s
    return _wrap(2.0 * math.atan2(qz, qw))


class YawEstimator:
    """Joy-Con 하나의 yaw (도). + = 오른쪽으로 돌림 (YAW_SIGN 참고). 평소에는 긴 축이 가리키는
    수평 방향, 긴 축이 많이 서면 보정 때 자세에서 수직축을 돈 각도 (모듈 docstring 참고).
    0 = 보정이 끝난 순간의 방향.

    상태: "idle" (보정 전, yaw 없음) -> start_calibration() -> "calibrating" -> "ready".
    add_report()는 HID 리포트마다 정확히 한 번 불러야 한다 (joyconrobotics 업데이트 훅).
    같은 리포트를 두 번 넣으면 두 번 적분된다. 시각은 인자로만 받는다 (시뮬레이터용)."""

    IDLE = "idle"
    CALIBRATING = "calibrating"
    READY = "ready"

    def __init__(self):
        self.state = self.IDLE
        self.bias = (0.0, 0.0, 0.0)  # rad/s, 보정에서 잰 자이로 바이어스
        self.calib_ok = False  # True = 가만히 있는 상태에서 잼. False = 실패가 이어져서 가장 나은 측정을 씀
        self.calib_tries = 0  # 이번 보정에서 실패한 측정 수 (YAW_CALIB_MAX_TRIES에 닿으면 가장 나은 것을 씀)
        self.calib_std = None  # 쓴 측정의 자이로 축별 표준편차 중 가장 큰 값 (rad/s)
        self.calib_accel_std = None  # 쓴 측정의 가속도 축별 표준편차 중 가장 큰 값 (g)
        self.calib_count = 0  # 끝난 보정 횟수 (텔레옵이 새 보정이 끝났는지 볼 때)
        # 중력축 단위벡터 (Joy-Con 좌표). 보정 전/중에는 가속도 로우패스로 따라간다 (split_heading용).
        self.up = _unit(YAW_FALLBACK_UP)
        self.rate_dps = 0.0  # 마지막 샘플의 yaw 속도 (도/초, 부호 적용 뒤)
        self.reports = 0  # 받은 리포트 수
        self.late_reports = 0  # 도착 간격이 YAW_REPORT_GAP_S를 넘은 횟수 (진단용, 위 설명 참고)
        self.bad_reports = 0  # NaN/inf가 들어 있어서 버린 리포트 수
        self.lib_busy_reports = 0  # joyconrobotics가 자체 바이어스를 재는 중이라 버린 리포트 수
        self._href = (1.0, 0.0, 0.0)  # 보정 때 정한 수평 기준 방향 (Joy-Con 좌표, 항상 up에 수직)
        self._cal_rows = None  # 보정 때 자세: (기준 방향, up x 기준 방향, up), Joy-Con 좌표
        self._psi_t = 0.0  # 보정 때 자세에서 수직축을 돈 각도 (rad, 감기 풀린 값, 부호 적용 전)
        self._psi_t_raw = 0.0
        self._yaw_offset = 0.0
        self._yaw_rad = 0.0
        self._grav_reject_s = 0.0  # 가속도 방향이 중력축과 계속 안 맞은 시간 (YAW_GRAVITY_RELAX_S)
        self._grav_relaxed = False
        self._calib_requested = False
        self._lib_busy = False
        self._last_report_t = None
        self._calib_best = None
        self._calib_last_up = None
        self._reset_calib_sums()

    def start_calibration(self):
        """바이어스를 (다시) 잰다. 다음 리포트부터 YAW_CALIB_S 동안 책상에 내려놓아야 한다.
        다른 스레드에서 불러도 된다 (플래그만 세우고, 실제 시작은 add_report가 한다).
        부른 순간부터 ready는 False, yaw_deg는 None이다."""
        self._calib_requested = True

    @property
    def ready(self):
        return self.state == self.READY and not self._calib_requested

    @property
    def yaw_deg(self):
        """보정이 끝난 뒤의 yaw (도). 보정 전/중에는 None. 0 = 보정이 끝난 순간의 방향
        (팔은 이 값을 그대로 쓰지 않고, 따라가기 시작한 순간의 값을 기준으로 쓴다)."""
        if not self.ready:
            return None
        return math.degrees(self._yaw_rad)

    def add_report(self, gyro_samples, accel_samples=None, now=None):
        """IMU 리포트 하나를 넣는다. gyro_samples: [(gx, gy, gz)] rad/s (실제 값, 샘플 3개,
        오래된 것부터). accel_samples: 같은 순서의 [(ax, ay, az)] g. 없으면 None (그러면
        중력축을 가속도계로 바로잡지 못한다). now: 리포트를 받은 시각 (초), 진단용.
        NaN/inf가 들어 있는 리포트는 통째로 버린다 (bad_reports)."""
        if not _finite_samples(gyro_samples) or (accel_samples is not None and not _finite_samples(accel_samples)):
            self.bad_reports += 1
            return
        if now is not None:
            if self._last_report_t is not None and now - self._last_report_t > YAW_REPORT_GAP_S:
                self.late_reports += 1
            self._last_report_t = now
        self.reports += 1
        if self._calib_requested:
            self._begin_calibration()  # 상태를 먼저 바꾼 뒤 플래그를 내린다 (ready가 잠깐도 True가 안 되게)
            self._calib_requested = False
        for i, g in enumerate(gyro_samples):
            a = accel_samples[i] if accel_samples is not None else None
            if self.state == self.READY:
                self._track_sample(g, a)
                continue
            self._follow_accel(a)
            if self.state == self.CALIBRATING:
                self._calib_sample(g, a)

    def add_joyconrobotics_report(self, device, now=None):
        """joyconrobotics 장치 핸들 (jc.gyro)의 지금 리포트를 넣는다. 그 핸들의 업데이트 훅
        안에서 부를 것 (훅은 리포트를 받은 리더 스레드에서, 다음 리포트가 오기 전에 불린다).
        gyro_in_rad의 절반 버그 (JOYCONROBOTICS_GYRO_RAD_FIX)와 GYRO_SCALE_FIX를 여기서 곱한다.
        joyconrobotics가 자체 바이어스를 재는 중이면 (device.is_calibrating) 그 리포트는 버리고,
        끝난 뒤 바이어스를 다시 잰다 (끝날 때 자이로 오프셋이 바뀌어서 이전 측정이 틀어진다).
        그동안 ready는 False다."""
        if getattr(device, "is_calibrating", False):
            if not self._lib_busy and self.state != self.IDLE:
                self._calib_requested = True  # ready를 바로 내린다
            self._lib_busy = True
            self.lib_busy_reports += 1
            return
        if self._lib_busy:
            self._lib_busy = False
            self._last_report_t = None  # 쉬는 동안의 간격을 늦은 리포트로 세지 않게
            if self.state != self.IDLE:
                self._calib_requested = True
        k = JOYCONROBOTICS_GYRO_RAD_FIX * GYRO_SCALE_FIX
        gyro = [(gx * k, gy * k, gz * k) for gx, gy, gz in device.gyro_in_rad]
        self.add_report(gyro, device.accel_in_g, now)

    def split_heading(self, g):
        """자이로 (gx, gy, gz)를 (yaw 변화율, 나머지)로 나눈다. 나머지 = g - 변화율 x 중력축:
        좌/우로 돌리기 (pan)를 뺀, 손목용 회전이다. 변화율은 YAW_SIGN 적용 전이고, 단위는 g 그대로
        (joyconrobotics의 절반 값을 넣으면 절반 값이 나온다). 바이어스는 빼지 않는다.
        텔레옵이 newcontrol 손목 적분기에 원시 자이로 대신 나머지를 넣으면 pan 동작이 손목으로
        새지 않는다. 긴 축을 도는 비틀기는 나머지에 그대로 남는다. 다만 긴 축이
        YAW_HEADING_FULL_DEG보다 많이 서면 비틀기의 일부가 pan 쪽으로 빠진다. 보정 전에는
        가속도 로우패스로 구한 중력축 (가속도가 없었으면 YAW_FALLBACK_UP)을 쓴다."""
        ux, uy, uz = up = self.up
        h = heading_rate(g, up)
        return h, (g[0] - h * ux, g[1] - h * uy, g[2] - h * uz)

    def _reset_calib_sums(self):
        self._n = 0
        self._sum_g = [0.0, 0.0, 0.0]
        self._sum_g2 = [0.0, 0.0, 0.0]
        self._sum_a = [0.0, 0.0, 0.0]
        self._sum_a2 = [0.0, 0.0, 0.0]
        self._n_a = 0

    def _begin_calibration(self):
        self.state = self.CALIBRATING
        self.calib_tries = 0
        self.calib_ok = False
        self.calib_std = None
        self.calib_accel_std = None
        self._calib_best = None
        self._calib_last_up = None
        self._reset_calib_sums()

    def _follow_accel(self, a):
        """보정 전/중: 중력축을 가속도 로우패스 (YAW_GRAVITY_LPF_TAU_S)로 따라간다."""
        if a is None:
            return
        norm = math.sqrt(_dot(a, a))
        if not YAW_GRAVITY_ACCEL_MIN_G <= norm <= YAW_GRAVITY_ACCEL_MAX_G:
            return
        k = IMU_SAMPLE_DT_S / YAW_GRAVITY_LPF_TAU_S
        u = self.up
        self.up = _unit(
            (u[0] + k * (a[0] / norm - u[0]), u[1] + k * (a[1] / norm - u[1]), u[2] + k * (a[2] / norm - u[2]))
        )

    def _calib_sample(self, g, a):
        for k in range(3):
            self._sum_g[k] += g[k]
            self._sum_g2[k] += g[k] * g[k]
        if a is not None:
            for k in range(3):
                self._sum_a[k] += a[k]
                self._sum_a2[k] += a[k] * a[k]
            self._n_a += 1
        self._n += 1
        if self._n < YAW_CALIB_SAMPLES:
            return
        n = self._n
        mean_g = tuple(s / n for s in self._sum_g)
        std_g = max(math.sqrt(max(0.0, s2 / n - m * m)) for s2, m in zip(self._sum_g2, mean_g))
        # 가속도: 들어온 게 없으면 판단하지 않는다 (중력축은 YAW_FALLBACK_UP).
        accel_ok, std_a = True, None
        if self._n_a:
            na = self._n_a
            mean_a = tuple(s / na for s in self._sum_a)
            std_a = max(math.sqrt(max(0.0, s2 / na - m * m)) for s2, m in zip(self._sum_a2, mean_a))
            norm = math.sqrt(sum(c * c for c in mean_a))
            if YAW_CALIB_ACCEL_MIN_G <= norm <= YAW_CALIB_ACCEL_MAX_G:
                # 중력축은 가장 최근 측정의 것을 쓴다 (실패가 이어져 예전 측정의 바이어스를 쓸 때도
                # 지금 자세에 가까운 쪽).
                self._calib_last_up = tuple(c / norm for c in mean_a)
                accel_ok = std_a <= YAW_CALIB_MAX_ACCEL_STD_G
            else:
                accel_ok = False
        # 실패가 이어질 때 쓸 후보: 가속도가 정상인 측정이 먼저, 그다음 자이로가 조용한 순서.
        key = (not accel_ok, std_g)
        if self._calib_best is None or key < self._calib_best[0]:
            self._calib_best = (key, mean_g, std_g, std_a)
        if accel_ok and std_g <= YAW_CALIB_MAX_STD_RADPS:
            self._finish_calibration(mean_g, std_g, std_a, ok=True)
            return
        self.calib_tries += 1
        if self.calib_tries >= YAW_CALIB_MAX_TRIES:
            _, best_bias, best_std_g, best_std_a = self._calib_best
            self._finish_calibration(best_bias, best_std_g, best_std_a, ok=False)
            return
        self._reset_calib_sums()

    def _finish_calibration(self, bias, std_g, std_a, ok):
        self.bias = bias
        if self._calib_last_up is not None:
            self.up = self._calib_last_up
        u = self.up  # (가속도가 없었으면 로우패스 값, 그것도 없으면 YAW_FALLBACK_UP)
        # 수평 기준 방향 = 지금 긴 축 (로컬 X)을 수평면에 내린 방향. 긴 축이 거의 수직이면 로컬 Y.
        href = None
        for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)):
            d = _dot(axis, u)
            v = (axis[0] - d * u[0], axis[1] - d * u[1], axis[2] - d * u[2])
            if _dot(v, v) > 0.1:
                href = _unit(v)
                break
        self._href = href
        self._cal_rows = (href, _cross(u, href), u)
        self._psi_t = self._psi_t_raw = 0.0
        self._grav_reject_s = 0.0
        self._grav_relaxed = False
        self._yaw_offset = 0.0
        self._yaw_offset = self._heading_angle()  # 보정이 끝난 순간 yaw = 0
        self._yaw_rad = 0.0
        self.calib_std = std_g
        self.calib_accel_std = std_a
        self.calib_ok = ok
        self._calib_best = None
        self._calib_last_up = None
        self._reset_calib_sums()
        self.calib_count += 1
        self.state = self.READY  # 마지막에: 다른 스레드가 READY를 보면 yaw는 이미 0

    def _heading_angle(self):
        """지금 자세의 yaw (rad, 부호 적용 전, _yaw_offset을 뺀 값). 긴 축 방향과 twist를 섞는다."""
        u, h = self.up, self._href
        e = _cross(u, h)
        # 1) 긴 축 (로컬 X)이 수평면에서 가리키는 방향: 기준 방향 h에서 중력축을 돈 각도.
        psi_x = math.atan2(e[0], h[0])
        # 2) 보정 때 자세에서 수직축을 돈 각도 (twist). 지금 자세의 행 (h, e, u)와 보정 때 행의
        #    내적이 상대 회전 행렬이다. 긴 축 방향은 twist에 가장 가까운 바퀴로 맞춘다 (따로 감으면
        #    긴 축이 수직 근처를 지날 때 바퀴 수가 틀어진다).
        m = tuple(tuple(_dot(r, c) for c in self._cal_rows) for r in (h, e, u))
        psi_t_raw = _twist_about_z(m)
        self._psi_t += _wrap(psi_t_raw - self._psi_t_raw)
        self._psi_t_raw = psi_t_raw
        return self._psi_t + _heading_weight(u) * _wrap(psi_x - self._psi_t) - self._yaw_offset

    def _track_sample(self, g, a):
        dt = IMU_SAMPLE_DT_S
        w = (g[0] - self.bias[0], g[1] - self.bias[1], g[2] - self.bias[2])
        u, h = self.up, self._href
        if YAW_TIGHTEN_RADPS > 0.0:
            v = _dot(w, u)  # 수직축 성분만 줄인다 (남은 바이어스는 수직축 드리프트로 나타남)
            if abs(v) < YAW_TIGHTEN_RADPS:
                cut = v - v * abs(v) / YAW_TIGHTEN_RADPS
                w = (w[0] - cut * u[0], w[1] - cut * u[1], w[2] - cut * u[2])
        # 세계에 고정된 방향 (중력축, 수평 기준 방향)은 Joy-Con 좌표에서 -w x v로 돈다.
        if YAW_GRAVITY_GYRO_PROPAGATE:
            c = _cross(w, u)
            u = (u[0] - dt * c[0], u[1] - dt * c[1], u[2] - dt * c[2])
        c = _cross(w, h)
        h = (h[0] - dt * c[0], h[1] - dt * c[1], h[2] - dt * c[2])
        # 중력축을 가속도계로 바로잡는다 (YAW_GRAVITY_* 설명 참고).
        if a is not None:
            norm = math.sqrt(_dot(a, a))
            if YAW_GRAVITY_ACCEL_MIN_G <= norm <= YAW_GRAVITY_ACCEL_MAX_G:
                an = (a[0] / norm, a[1] / norm, a[2] / norm)
                k = 0.0
                if not YAW_GRAVITY_GYRO_PROPAGATE:
                    k = dt / YAW_GRAVITY_LPF_TAU_S
                elif _dot(w, w) <= YAW_GRAVITY_MAX_RATE_RADPS * YAW_GRAVITY_MAX_RATE_RADPS:
                    inside = _dot(an, _unit(u)) >= _GRAVITY_GATE_COS
                    if inside:
                        self._grav_reject_s = 0.0
                        self._grav_relaxed = False
                    elif not self._grav_relaxed:
                        self._grav_reject_s += dt
                        self._grav_relaxed = self._grav_reject_s >= YAW_GRAVITY_RELAX_S
                    if inside or self._grav_relaxed:
                        k = dt / YAW_GRAVITY_TAU_S
                if k:
                    u = (u[0] + k * (an[0] - u[0]), u[1] + k * (an[1] - u[1]), u[2] + k * (an[2] - u[2]))
        n = math.sqrt(_dot(u, u))
        u = (u[0] / n, u[1] / n, u[2] / n) if n > 1e-9 else self.up
        # 수평 기준 방향을 중력축에 수직으로 다시 맞춘다.
        d = _dot(h, u)
        h = (h[0] - d * u[0], h[1] - d * u[1], h[2] - d * u[2])
        nh = math.sqrt(_dot(h, h))
        h = (h[0] / nh, h[1] / nh, h[2] / nh) if nh > 1e-9 else self._href
        self.up, self._href = u, h
        yaw = YAW_SIGN * self._heading_angle()
        self.rate_dps = math.degrees(yaw - self._yaw_rad) / dt
        self._yaw_rad = yaw


@dataclass(frozen=True)
class ArmCommand:
    """step() 한 번의 결과 (불변). 관절 목표는 도, 로봇 관절 기준."""

    pan_deg: float  # shoulder_pan 목표
    lift_deg: float  # shoulder_lift 목표
    elbow_deg: float  # elbow_flex 목표
    # 그리퍼 각도 유지 보정 = -(lift 변화) - (elbow 변화), set_home에 넘긴 실제 시작 자세 기준
    # (newcontrol의 KEEP_GRIPPER_ANGLE 식). 손목 자이로 값에 더해 wrist_flex로 보낸다. 손목 자체는 이 모델 밖.
    wrist_flex_comp_deg: float
    x_m: float  # 사용자가 움직인 그리퍼 끝 목표 (팔 평면, shoulder_lift 축 기준, m). 출력은 속도 제한 때문에 잠깐 늦을 수 있음
    z_m: float
    yaw_deg: float | None  # pan에 쓴 yaw: 지금 기준점 대비 (= (pan 목표 - home_pan) / 배율). 따라가지 않았으면 None
    engaged: bool  # pan이 yaw를 따라가는 중
    pan_at_limit: bool  # pan 목표가 한계에 닿아 있다 (PAN_AT_LIMIT_RELEASE_DEG 안쪽으로 들어와야 꺼짐)
    reach_blocked: bool  # 이번 틱의 앞/뒤 움직임을 한계 때문에 버렸다
    height_blocked: bool  # 이번 틱의 위/아래 움직임을 한계 때문에 버렸다
    lagging: bool  # 출력이 아직 목표를 따라가는 중 (속도 제한)


class StickYawArm:
    """팔 하나의 동작 모델: 스틱 (가로, 세로)과 yaw -> shoulder_pan / shoulder_lift / elbow_flex.

    한계는 (최소, 최대) 도, 로봇 관절 기준 (텔레옵의 캘리브레이션 한계에서 여유를 뺀 값).
    lift/elbow는 IK가 정확한 범위 (IK_*_DEG)와 겹치는 쪽으로 더 좁힌다.
    *_cmd = 사용자가 지금 가리키는 목표, *_out = 로봇으로 보내는 목표 (cmd를 속도 제한으로 따라감).
    set_home()을 먼저 불러야 한다 (그 전의 step()은 아무것도 안 하고 None)."""

    def __init__(self, side, pan_limits_deg, lift_limits_deg, elbow_limits_deg):
        if side not in STICK_HEIGHT_SIGN:
            raise ValueError(f"side must be 'right' or 'left', got {side!r}")
        self.side = side
        self.pan_lo, self.pan_hi = pan_limits_deg
        self.lift_lo = lift_limits_deg[0]
        self.lift_hi = min(lift_limits_deg[1], IK_LIFT_MAX_DEG)
        self.elbow_lo = max(elbow_limits_deg[0], IK_ELBOW_MIN_DEG)
        self.elbow_hi = min(elbow_limits_deg[1], IK_ELBOW_MAX_DEG)
        for name, lo, hi in (
            ("pan", self.pan_lo, self.pan_hi),
            ("lift", self.lift_lo, self.lift_hi),
            ("elbow", self.elbow_lo, self.elbow_hi),
        ):
            if not lo < hi:  # NaN 한계도 여기서 걸림
                raise ValueError(f"[{side}] empty {name} range: {lo} .. {hi} deg")
        self.height_sign = STICK_HEIGHT_SIGN[side]
        self.home_pan = self.home_lift = self.home_elbow = 0.0
        self.comp_ref_lift = self.comp_ref_elbow = 0.0  # 그리퍼 각도 보정의 기준 (실제 시작 자세)
        self.pan_cmd = self.lift_cmd = self.elbow_cmd = 0.0
        self.x, self.z = so101_fk(0.0, 0.0)  # 지금 목표점 (m), 항상 so101_fk(lift_cmd, elbow_cmd)
        self.pan_out = self.lift_out = self.elbow_out = 0.0
        # pan = home_pan + YAW_PAN_SIGN * YAW_TO_PAN_GAIN * (yaw - yaw_ref). None = 다음 yaw에서 다시 잡음
        self.yaw_ref = None
        self.engaged = False  # pan이 yaw를 따라가나
        self.engage_count = 0  # 따라가기 시작한 횟수 (텔레옵이 손목 적분기를 0으로 맞출 때)
        self._engage_req = False
        self._stick_rearmed = False  # 스틱이 가운데로 한 번 돌아왔나 (ENGAGE_ON_STICK의 조건)
        self._home_req = False
        self._rezero_req = False
        self._at_limit = False
        self.command = None  # 마지막 ArmCommand (set_home 전에는 None)

    def set_home(self, pan_deg, lift_deg, elbow_deg):
        """시작 자세 (로봇의 지금 관절값)를 기억하고 거기서 시작한다. 자세가 한계나 IK 범위
        밖이면 범위 안의 가장 가까운 자세를 home으로 삼고, 출력은 지금 자세에서 속도 제한으로
        천천히 그리로 간다 (그동안 그리퍼 각도 보정이 손목을 맞춰서 그리퍼 각도는 시작 그대로).
        pan은 아직 yaw를 따라가지 않는다 (engaged = False). 범위 밖이었으면 True를 돌려준다
        (텔레옵이 경고를 찍는 용도). step()을 돌리기 전에 부른다."""
        if not all(math.isfinite(v) for v in (pan_deg, lift_deg, elbow_deg)):
            raise ValueError(f"[{self.side}] start pose must be finite: {pan_deg}, {lift_deg}, {elbow_deg}")
        self.home_pan = _clamp(pan_deg, self.pan_lo, self.pan_hi)
        self.home_lift = _clamp(lift_deg, self.lift_lo, self.lift_hi)
        self.home_elbow = _clamp(elbow_deg, self.elbow_lo, self.elbow_hi)
        self.comp_ref_lift, self.comp_ref_elbow = lift_deg, elbow_deg
        self.pan_cmd = self.home_pan
        self._set_cmd_joints(self.home_lift, self.home_elbow)
        self.pan_out, self.lift_out, self.elbow_out = pan_deg, lift_deg, elbow_deg
        self.yaw_ref = None
        self.engaged = False
        self._engage_req = self._stick_rearmed = False
        self._home_req = self._rezero_req = False
        self._at_limit = not (
            self.pan_lo + PAN_AT_LIMIT_RELEASE_DEG < self.pan_cmd < self.pan_hi - PAN_AT_LIMIT_RELEASE_DEG
        )
        self.command = self._make_command(None, False, False)
        return (self.home_pan, self.home_lift, self.home_elbow) != (pan_deg, lift_deg, elbow_deg)

    def go_home(self):
        """목표를 시작 자세로 (pan 포함). 출력은 속도 제한으로 천천히 간다. yaw 기준점은 지금
        Joy-Con 방향으로 다시 잡힌다 (= 지금 손 방향이 home 방향). 다음 step()이 처리한다
        (다른 스레드에서 불러도 됨)."""
        self._home_req = True

    def rezero_yaw(self):
        """팔은 그대로 두고, 지금 손 방향을 지금 pan으로 다시 맞춘다 (yaw 드리프트나 손을 돌릴
        범위가 모자랄 때). 다음 step()이 처리한다 (다른 스레드에서 불러도 됨)."""
        self._rezero_req = True

    def engage(self):
        """pan이 yaw를 따라가기 시작하게 한다. yaw가 있는 다음 step()에서, 그 순간의 손 방향 =
        지금 pan으로 잡는다 (yaw가 없으면 들어올 때까지 미뤄진다). 이미 따라가는 중이면 아무 일도
        없다 (그때는 rezero_yaw). 다른 스레드에서 불러도 됨."""
        self._engage_req = True

    def disengage(self):
        """pan이 yaw를 따라가지 않는다 (Joy-Con을 내려놓을 때, 바이어스를 다시 잴 때). 바로
        적용된다. ENGAGE_ON_STICK이면 스틱이 한 번 가운데로 돌아왔다가 다시 밀릴 때 (그리고 yaw가
        있을 때) 다시 따라가기 시작한다."""
        self._engage_req = False
        self._stick_rearmed = False
        self.engaged = False

    def step(self, dt, stick_x, stick_y, yaw_deg):
        """한 틱. dt: 지난 step 이후 초 (NaN/inf/음수 = 0, MAX_STEP_DT_S로 자름). stick_x / stick_y:
        스틱 가로/세로 (-1..1, + = 오른쪽/위). 둘 중 하나라도 None이면 그리퍼 끝은 그대로.
        yaw_deg: YawEstimator.yaw_deg. None/NaN/inf면 pan 목표는 그대로 (그리고 다음 yaw에서
        기준점을 다시 잡음). 새 ArmCommand를 돌려주고 self.command에도 둔다. set_home 전이면 None."""
        if self.command is None:
            return None
        dt = min(dt, MAX_STEP_DT_S) if 0.0 < dt < math.inf else 0.0  # NaN도 0
        yaw_ok = yaw_deg is not None and math.isfinite(yaw_deg)

        ref = self.yaw_ref
        if self._home_req:
            self._home_req = False
            self.pan_cmd = self.home_pan
            self._set_cmd_joints(self.home_lift, self.home_elbow)
            ref = None
        if self._rezero_req:
            self._rezero_req = False
            ref = None

        # 스틱 -> 그리퍼 끝 앞/뒤, 위/아래 (IK). 움직일 때만 IK를 다시 푼다.
        reach_blocked = height_blocked = stick_push = False
        if stick_x is not None and stick_y is not None:
            rate = dict(zip(STICK_AXES, (stick_to_rate(stick_x), stick_to_rate(stick_y))))
            stick_push = bool(rate["reach"] or rate["height"])
            if not stick_push:
                self._stick_rearmed = True
            dx = IK_REACH_MPS * STICK_REACH_SIGN * rate["reach"] * dt
            dz = IK_HEIGHT_MPS * self.height_sign * rate["height"] * dt
            if dx or dz:
                moved_x, moved_z = self._move_point(dx, dz)
                reach_blocked = dx != 0.0 and not moved_x
                height_blocked = dz != 0.0 and not moved_z

        # 따라가기 시작: yaw가 있을 때만 (그래야 그 순간의 손 방향을 잡는다).
        if (
            not self.engaged
            and yaw_ok
            and (self._engage_req or (ENGAGE_ON_STICK and stick_push and self._stick_rearmed))
        ):
            self.engaged = True
            self._engage_req = False
            self.engage_count += 1
            ref = None

        # yaw -> pan (위치 제어).
        clamped = False
        yaw_used = None
        if not self.engaged or not yaw_ok:
            ref = None
        else:
            k = YAW_PAN_SIGN * YAW_TO_PAN_GAIN
            if ref is None:
                # 지금 pan 목표에서 이어간다 (손이 어느 쪽을 보고 있든 팔이 튀지 않게).
                ref = yaw_deg - (self.pan_cmd - self.home_pan) / k
            pan = self.home_pan + k * (yaw_deg - ref)
            if not self.pan_lo <= pan <= self.pan_hi:
                clamped = True
                pan = _clamp(pan, self.pan_lo, self.pan_hi)
                if PAN_DRAG_AT_LIMIT:
                    ref = yaw_deg - (pan - self.home_pan) / k
            self.pan_cmd = pan
            yaw_used = yaw_deg - ref
        self.yaw_ref = ref
        if clamped:
            self._at_limit = True
        elif self.pan_lo + PAN_AT_LIMIT_RELEASE_DEG < self.pan_cmd < self.pan_hi - PAN_AT_LIMIT_RELEASE_DEG:
            self._at_limit = False

        # 출력은 목표를 속도 제한으로 따라간다. lift와 elbow는 같은 비율로 줄여서 관절 공간에서
        # 곧게 간다 (한 관절만 먼저 가지 않음). 평소 스틱 이동의 작은 지연에서는 그리퍼 끝도 거의
        # 곧지만, home이나 시작 자세 보정처럼 크게 따라갈 때는 그리퍼 끝 경로가 몇 cm 휜다.
        self.pan_out = _approach(self.pan_out, self.pan_cmd, PAN_MAX_DEG_S * dt)
        d_lift, d_elbow = self.lift_cmd - self.lift_out, self.elbow_cmd - self.elbow_out
        biggest = max(abs(d_lift), abs(d_elbow))
        max_step = LIFT_ELBOW_MAX_DEG_S * dt
        if biggest <= max_step:
            self.lift_out, self.elbow_out = self.lift_cmd, self.elbow_cmd
        else:
            s = max_step / biggest
            self.lift_out += d_lift * s
            self.elbow_out += d_elbow * s

        self.command = self._make_command(yaw_used, reach_blocked, height_blocked)
        return self.command

    def debug_str(self):
        c = self.command
        if c is None:
            return f"{self.side}: (before set_home)"
        yaw = " off  " if c.yaw_deg is None else f"{c.yaw_deg:+6.1f}"
        flags = "".join(
            f for f, on in (("P", c.pan_at_limit), ("R", c.reach_blocked), ("H", c.height_blocked), ("~", c.lagging)) if on
        )
        return (
            f"{self.side}: x={c.x_m * 100:5.1f} z={c.z_m * 100:5.1f} cm yaw={yaw} pan={c.pan_deg:+6.1f} "
            f"lift={c.lift_deg:+6.1f} elbow={c.elbow_deg:+6.1f} comp={c.wrist_flex_comp_deg:+6.1f} {flags}"
        )

    def _set_cmd_joints(self, lift, elbow):
        """관절 목표를 바로 정하고 목표점을 그 자세의 위치로 맞춘다 (home용; 스틱 이동은 _move_point)."""
        self.lift_cmd, self.elbow_cmd = lift, elbow
        self.x, self.z = so101_fk(lift, elbow)

    def _move_point(self, dx, dz):
        """목표점을 (dx, dz)만큼 옮긴다 (newcontrol의 _move_point 그대로). 옮긴 점이 닿지 않거나
        관절 한계를 넘으면 그 움직임은 버리고 제자리 (잘라서 맞추면 앞으로 밀 때 그리퍼가 아래로
        미끄러진다). 두 축을 같이 움직이다 한쪽만 막히면 다른 쪽은 간다.
        (앞/뒤가 움직였나, 위/아래가 움직였나)를 돌려준다."""
        for ddx, ddz in ((dx, dz), (dx, 0.0), (0.0, dz)):
            if ddx == 0.0 and ddz == 0.0:
                continue
            x, z = self.x + ddx, self.z + ddz
            if x < IK_X_MIN_M and ddx < 0.0:
                continue  # 너무 뒤 (IK_X_MIN_M). 앞으로 가는 건 언제나 허용
            lift, elbow = so101_ik(x, z)
            if not (self.lift_lo <= lift <= self.lift_hi and self.elbow_lo <= elbow <= self.elbow_hi):
                continue  # 관절 한계
            fx, fz = so101_fk(lift, elbow)
            if abs(fx - x) > 1e-6 or abs(fz - z) > 1e-6:
                continue  # 닿지 않는 점 (IK가 작업 공간 경계로 당겼거나 URDF 한계로 잘랐음)
            self.lift_cmd, self.elbow_cmd, self.x, self.z = lift, elbow, x, z
            return ddx != 0.0, ddz != 0.0
        return False, False

    def _make_command(self, yaw_used, reach_blocked, height_blocked):
        return ArmCommand(
            pan_deg=self.pan_out,
            lift_deg=self.lift_out,
            elbow_deg=self.elbow_out,
            wrist_flex_comp_deg=-(self.lift_out - self.comp_ref_lift) - (self.elbow_out - self.comp_ref_elbow),
            x_m=self.x,
            z_m=self.z,
            yaw_deg=yaw_used,
            engaged=self.engaged,
            pan_at_limit=self._at_limit,
            reach_blocked=reach_blocked,
            height_blocked=height_blocked,
            lagging=(
                self.pan_out != self.pan_cmd or self.lift_out != self.lift_cmd or self.elbow_out != self.elbow_cmd
            ),
        )


if __name__ == "__main__":
    print(
        "xlerobot_JoyCon_1008.py is the motion model module (stick + Joy-Con yaw -> arm motors 1-3), "
        "not a runnable teleop. Import it from a Final_* teleop or a simulator; see the docstring."
    )
