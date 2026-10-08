#!/usr/bin/env python3
"""XLeRobot 카메라 3대 (헤드 + 양 손목) 라이브 뷰 + YOLOE 물체 인식.

2026-10-08 버전 (xlerobot_camera_1008.py). 텔레옵 스크립트와 따로 돌아가는 카메라 전용
테스트 스크립트다. 로봇 버스나 Joy-Con은 건드리지 않는다. 검증이 끝나면 Final 텔레옵에
합친다 (Final_1007c / Final_1007_newcontrol 안의 카메라 코드는 아직 1007b 그대로).

1008에서 바뀐 것 (2026-10-08, 사용자 요청):
  - 물체를 박스 대신 윤곽선으로 그린다. 모델 (YOLOE-26s-seg)이 내놓는 물체 마스크에서
    바깥 윤곽을 뽑아 따라 그린다. m 키로 예전 박스 표시와 바꿔 볼 수 있다.
    ultralytics의 masks.xy는 한 물체가 여러 조각으로 나뉘면 조각들을 선으로 이어 붙인
    폴리곤 하나를 주므로 (masks2segments, strategy="all"), 그대로 그리면 조각 사이에 엉뚱한
    선이 생긴다. 그래서 마스크에서 바깥 윤곽을 조각별로 직접 뽑는다 (OUTLINE_* 참고).
  - 블랙리스트에서 chair를 뺐다 (BLOCK_WORDS = desk만). 의자가 화면에 나오게 하려고 chair를
    찾을 단어 (PROMPT_WORDS)에 넣었다 (15개 -> 16개). 블랙리스트에서만 빼면 chair가 단어
    목록에서 아예 빠져서, 의자가 안 잡히거나 다른 단어로 잡힌다.
  - 카메라당 표시 개수 MAX_OBJECTS_PER_CAMERA 10 -> 15.
  - 인식 빈도를 DETECT_HZ (15 Hz)로 제한한다 ("확인 빈도를 좀 늦추자", 사용자). 1007b까지는
    화면 루프가 돌 때마다 인식해서 GPU가 되는 만큼 (약 25-35 Hz, 카메라 30fps에 거의 붙어서)
    돌았다. 화면은 지금처럼 새 카메라 프레임마다 그리고, 그 사이에는 마지막 인식 결과
    (윤곽선)를 그린다. --detect-hz로 바꾸고, 0이면 예전처럼 제한 없음. 화면 위 상태 줄에
    실제 인식 빈도가 나온다.
  - 화면 정리 (StableObjects)의 "몇 번 잡혀야 나타남 / 몇 번 놓쳐야 사라짐"을 초 단위
    (STABLE_SHOW_AFTER_S, STABLE_HOLD_S)로 바꿨다. 인식 빈도를 바꿔도 나타나고 사라지는
    느낌이 같다 (예전 3번 / 6번 @ 약 25 Hz = 약 0.12초 / 0.25초).

환경: 텔레옵의 .venv와 별도인 .venv-vision (torch CUDA 빌드 + ultralytics).
텔레옵 .venv의 torch는 CPU 전용이고 lerobot이 버전을 묶고 있어서 나눴다.
RTX 50 시리즈는 CUDA 12.8 이상 빌드가 필요하다.

    uv venv .venv-vision --python 3.12
    uv pip install --python .venv-vision/Scripts/python.exe torch torchvision --index-url https://download.pytorch.org/whl/cu128
    uv pip install --python .venv-vision/Scripts/python.exe ultralytics opencv-python

실행 (작업 레포 폴더에서):

    .venv-vision\\Scripts\\python xlerobot_camera_1008.py --list          # 열리는 카메라 번호와 밝기만
    .venv-vision\\Scripts\\python xlerobot_camera_1008.py                 # 3대 + 인식 (PROMPT_WORDS 16개)
    .venv-vision\\Scripts\\python xlerobot_camera_1008.py --prompt "cup,bottle,phone"   # 찾을 단어 바꾸기
    .venv-vision\\Scripts\\python xlerobot_camera_1008.py --block "desk,box"            # 블랙리스트 바꾸기
    .venv-vision\\Scripts\\python xlerobot_camera_1008.py --block ""                    # 블랙리스트 끄기
    .venv-vision\\Scripts\\python xlerobot_camera_1008.py --no-detect     # 카메라만 (인식 끔)
    .venv-vision\\Scripts\\python xlerobot_camera_1008.py --cams head     # 헤드만
    .venv-vision\\Scripts\\python xlerobot_camera_1008.py --head 0 --left 4 --right 2   # 번호 지정

키 (영상 창이 선택된 상태에서):
    q / Esc  종료
    d        인식 켜기 / 끄기
    m        윤곽선 / 박스 표시 바꾸기 (기본: 윤곽선)
    s        스냅샷 저장 (captures/ 폴더: 카메라별 원본 + 표시 화면)

카메라 번호 (같은 모델 USB2.0_CAM1 세 대, 640x480 YUY2 약 30fps):
    2026-10-07 실측 (기본값): 헤드 = 0, 왼팔 손목 = 4, 오른팔 손목 = 2 (처음 2/4로 적었다가 사용자 확인으로 좌우 바꿈).
    2026-10-06 실측: 헤드 = 4, 왼팔 손목 = 2, 오른팔 손목 = 3, 0 = 노트북 내장, 1 = 가상 카메라.
Windows는 꽂는 순서나 재부팅에 따라 번호를 다시 매긴다. 세 대가 같은 모델이라
이름으로는 구분이 안 된다. 화면이 엉뚱하면 --list로 번호를 보고 (렌즈 하나를
손으로 가리면 그 번호의 밝기가 뚝 떨어진다) --head/--left/--right로 지정한다.

인식 모델: YOLOE-26s-seg (ultralytics), 단어 지정 모드 (YOLOE_WEIGHTS + 텍스트 임베딩).
박스와 마스크 (윤곽선)가 한 번에 나온다. 가중치는 처음 실행할 때 자동으로 내려받는다.

1007b에서 바뀐 것 (2026-10-07, 모델 비교 후 사용자 결정):
  - 프롬프트 없는 모드(내장 단어 4585개)를 버리고 단어 지정 모드만 쓴다.
    xlerobot_model_compare_1007.py로 오늘 스냅샷 9장을 비교했을 때, 단어
    지정 모드는 벽 오인식이 없었고 의자, 선풍기, 램프는 그대로 잡았고 더
    빨랐다 (평균 18 ms vs 28 ms). 프롬프트 없는 모드가 필요하면 1007 / 1006 파일.
  - 찾을 단어 PROMPT_WORDS 15개 (--prompt로 바꿈). (1008: chair를 넣어 16개)
  - 블랙리스트 BLOCK_WORDS (1007b: desk, chair. 1008: desk만. --block으로 바꿈): 모델에는 단어로
    넣되 화면에는 안 보인다. 단어에서 아예 빼면 모델이 의자를 다른 단어(box 등)로
    잘못 잡을 수 있어서, 의자는 "chair"로 잡게 두고 NMS가 끝난 뒤에 버린다.
    (1007의 블랙리스트는 반대로 NMS 전에 걸렀다. 거기서는 엉뚱한 단어가 진짜 물체
    박스를 지우는 걸 막는 게 목적이었고, 여기서는 막은 단어가 그 물체를 차지하게
    하는 게 목적이다.)
  - 1007의 화이트리스트, --find, 큰 박스 거르기는 뺐다 (단어 지정 모드에서는 필요 없음).
  - 신뢰도 하한 0.4 -> 0.3 (단어 지정 모드는 점수가 낮게 나온다. 비교 때 쓴 값).

화면 정리 (2026-10-07, 첫 실행 피드백: "오락가락하고 너무 많이 잡아서 정신없음"):
  - 신뢰도 하한 0.25 -> 0.4, 한 물체에 겹친 박스는 이름이 달라도 하나만 남김
    (agnostic NMS), 물체가 아닌 단어 (IGNORE_WORDS) 버림. (1007b: 하한 0.3, 아래 참고)
  - StableObjects: 프레임마다 새로 판단하던 걸 화면 위 물체로 이어 붙인다.
    여러 번 잡힌 물체만 보여주고 (1008: STABLE_SHOW_AFTER_S), 한두 번 놓쳐도
    잠깐 유지한다 (1008: STABLE_HOLD_S). 이름은 최근에 가장 자주/강하게
    붙은 것으로 고정해서 chair <-> office chair 처럼 바뀌는 걸 막는다.
  - 카메라당 MAX_OBJECTS_PER_CAMERA 개까지만 표시 (1008: 10 -> 15).
"""

import argparse
import os
import sys
import threading
import time
import zlib
from datetime import datetime

import cv2
import numpy as np

CAMERA_DEFAULTS = {"head": 0, "left": 4, "right": 2}  # 2026-10-07 번호 (처음 왼 2 / 오른 4로 적었다가 사용자 확인으로 좌우 바꿈)
CAMERA_ORDER = ("left", "head", "right")  # 화면 배치: 왼손목 | 헤드 | 오른손목
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# YOLOE-26 small (ultralytics 8.4.173 기준 최신). 더 정확하게: m / l, 더 빠르게: n.
YOLOE_WEIGHTS = "yoloe-26s-seg.pt"  # 단어 지정 모드 (1007b부터 이것만)
# 찾을 단어 16개 (1007b에서 모델 비교 때 20개에서 15개로 줄임: table, cable, robot arm 빼고
# chair, desk는 블랙리스트로. 1008: chair를 블랙리스트에서 빼고 여기 넣음). 소문자 영어.
# --prompt로 바꾼다.
PROMPT_WORDS = (
    "person", "cup", "bottle", "cell phone", "box", "bag", "book", "scissors", "pen",
    "laptop", "keyboard", "mouse", "monitor", "fan", "lamp", "chair",
)
# 블랙리스트: 모델에는 단어로 넣지만 화면에는 안 보이는 것 (docstring "1007b" 참고).
# --block으로 바꾼다. (1008: chair를 뺌, 사용자 요청)
BLOCK_WORDS = ("desk",)
DETECT_CONF = 0.3  # 이보다 낮은 신뢰도의 검출은 버림 (1006: 0.25 -> 0.4, 1007b: 단어 지정 모드라 0.3)
DETECT_IMGSZ = 640
DETECT_MAX_DET = 30  # 모델이 한 프레임에서 내놓는 최대 개수 (표시 전, 필터 여유분)
# 윤곽선 (1008). 마스크는 원본 해상도로 받는다 (retina_masks: 화면 좌표와 그대로 맞음).
OUTLINE_MIN_AREA_FRAC = 0.02  # 물체 마스크에서 이보다 작은 조각 (가장 큰 조각 대비 넓이)은 버림 (점 같은 부스러기)
OUTLINE_SIMPLIFY_PX = 1.5  # 윤곽선 점 줄이기 (cv2.approxPolyDP 허용 오차, 픽셀). 계단 모양과 떨림 감소
OUTLINE_THICKNESS = 2

# 인식 빈도 (1008, 모듈 docstring 참고). 화면은 새 카메라 프레임마다 그린다 (약 30fps).
DETECT_HZ = 15.0  # 인식 빈도 상한 (Hz). --detect-hz로 바꿈, 0 = 제한 없음 (1007b처럼 화면마다)
# 화면 정리 (모듈 docstring 참고). 1008부터 초 단위: 인식 빈도에 맞춰 횟수로 바꾼다 (stable_counts).
STABLE_SHOW_AFTER_S = 0.12  # 이 시간 동안의 인식에서 잡혀야 화면에 나타남 (1007b: 3번 @ 약 25 Hz)
STABLE_HOLD_S = 0.25  # 이 시간 동안 연속으로 놓쳐야 화면에서 사라짐 (1007b: 6번 @ 약 25 Hz)
STABLE_MIN_HITS_FLOOR = 2  # 인식이 느려도 적어도 이만큼은 잡혀야 나타남 (한 번 튄 오인식 거르기)
STABLE_MATCH_IOU = 0.3  # 이전 물체와 박스가 이만큼 겹치면 같은 물체로 봄
STABLE_BOX_SMOOTH = 0.5  # 박스 위치 = 이전 * 이 값 + 새 값 * (1 - 이 값). 떨림 감소
STABLE_LABEL_DECAY = 0.9  # 이름 점수가 프레임마다 이만큼 줄어듦 (오래된 이름은 잊힘)
MAX_OBJECTS_PER_CAMERA = 15  # 1008: 10 -> 15 (사용자 요청)

WINDOW_NAME = "xlerobot cameras"
MAX_WINDOW_WIDTH = 1800  # 세 화면을 붙인 폭이 이보다 크면 줄여서 표시
LABEL_PRINT_INTERVAL_S = 2.0  # 터미널에 카메라별 인식 결과를 찍는 간격
CAPTURE_DIR = "captures"

BACKEND = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY


class CameraReader:
    """카메라 하나를 자기 스레드에서 계속 읽고 가장 최근 프레임만 들고 있다.
    인식이 카메라보다 느려도 옛 프레임이 버퍼에 쌓여 화면이 밀리지 않게 하려는 것."""

    def __init__(self, name, index, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
        self.name = name
        self.index = index
        self.cap = cv2.VideoCapture(index, BACKEND)
        if not self.cap.isOpened():
            raise RuntimeError(f"camera {name} (index {index}) did not open")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._lock = threading.Lock()
        self._frame = None
        self.frame_id = 0
        self.fps = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, name=f"cam-{name}", daemon=True)
        self._thread.start()

    def _loop(self):
        n, t0 = 0, time.monotonic()
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            with self._lock:
                self._frame = frame
                self.frame_id += 1
            n += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.fps = n / (now - t0)
                n, t0 = 0, now

    def latest(self):
        """(가장 최근 프레임 또는 None, 프레임 번호)."""
        with self._lock:
            return self._frame, self.frame_id

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.cap.release()


class Detector:
    """YOLOE 래퍼. 프레임 리스트를 한 번에 (배치로) 넣고 Results 리스트를 받는다."""

    def __init__(self, prompt_words, device, conf=DETECT_CONF, imgsz=DETECT_IMGSZ, block_words=()):
        from ultralytics import YOLOE  # 인식을 끄면 import도 안 하게 여기서

        self.conf = conf
        self.imgsz = imgsz
        self.device = device
        self.weights = YOLOE_WEIGHTS
        self.model = YOLOE(self.weights)
        # 블랙리스트 단어도 모델에는 넣는다 (그 물체를 그 단어가 차지하게). 화면에서만 뺀다.
        self.hidden = {w.lower() for w in block_words}
        classes = list(prompt_words) + [w for w in block_words if w.lower() not in {p.lower() for p in prompt_words}]
        self.model.set_classes(classes)  # 텍스트 임베딩은 ultralytics가 계산
        shown = [w for w in classes if w.lower() not in self.hidden]
        self.mode = f"{len(shown)} words" + (f", block {', '.join(sorted(self.hidden))}" if self.hidden else "")
        self.shown = shown

    def __call__(self, frames):
        """프레임 리스트 -> 프레임마다 [(이름, 신뢰도, 박스 xyxy, 윤곽선 목록 또는 None), ...]."""
        results = self.model.predict(
            frames, conf=self.conf, imgsz=self.imgsz, device=self.device, max_det=DETECT_MAX_DET,
            agnostic_nms=True, retina_masks=True, verbose=False,
        )
        return [extract_detections(r, self.hidden) for r in results]


def mask_outlines(mask, offset=(0, 0)):
    """마스크 (H, W, uint8 0/1) -> 바깥 윤곽 목록 [(K, 1, 2) int32] 또는 None. 가장 큰 조각 대비
    OUTLINE_MIN_AREA_FRAC보다 작은 조각은 버리고, 점을 OUTLINE_SIMPLIFY_PX로 줄인다.
    offset = 마스크 조각을 잘라낸 위치 (x, y). 윤곽 좌표에 더해져 원본 화면 좌표가 된다."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=offset)
    if not contours:
        return None
    areas = [cv2.contourArea(c) for c in contours]
    biggest = max(areas)
    if biggest <= 0.0:
        return None  # 넓이가 없는 선 모양 부스러기뿐
    kept = [
        cv2.approxPolyDP(c, OUTLINE_SIMPLIFY_PX, True)
        for c, area in zip(contours, areas)
        if area >= biggest * OUTLINE_MIN_AREA_FRAC
    ]
    kept = [c for c in kept if len(c) >= 3]
    return kept or None


def extract_detections(result, hidden=()):
    """ultralytics Results -> [(이름, 신뢰도, 박스 xyxy (np), 윤곽선 목록 또는 None)].
    hidden(블랙리스트)에 있는 이름은 버린다. NMS가 끝난 뒤라서, 그 박스가 겹친
    다른 단어의 박스를 이미 지운 상태다 (의자를 의자로 잡고 통째로 숨김).
    윤곽선 (1008): 남는 검출의 마스크만, 박스 안쪽 부분만 잘라서 CPU로 가져와 윤곽을 뽑는다
    (마스크 전체를 옮기지 않아서 빠르다). 마스크는 retina_masks라 원본 해상도다."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []
    names = result.names
    xyxy = boxes.xyxy.cpu().numpy()
    masks = result.masks.data if result.masks is not None else None
    if masks is not None and tuple(masks.shape[1:]) != tuple(result.orig_shape[:2]):
        masks = None  # 해상도가 다르면 (retina_masks가 안 먹은 경우) 윤곽 없이 박스만
    dets = []
    for i, (c, p, box) in enumerate(zip(boxes.cls.tolist(), boxes.conf.tolist(), xyxy)):
        name = names[int(c)]
        if name.lower() in hidden:
            continue
        outline = None
        if masks is not None:
            h, w = masks.shape[1:]
            x1, y1 = max(0, int(box[0]) - 2), max(0, int(box[1]) - 2)
            x2, y2 = min(w, int(box[2]) + 3), min(h, int(box[3]) + 3)
            if x2 > x1 and y2 > y1:
                crop = (masks[i, y1:y2, x1:x2] > 0.5).byte().cpu().numpy()
                outline = mask_outlines(crop, offset=(x1, y1))
        dets.append((name, p, box, outline))
    return dets


def box_iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


class StableObject:
    """화면 위 물체 하나. 프레임마다의 검출을 이어 붙인 것."""

    def __init__(self, name, conf, box, outline):
        self.box = box.astype(float)
        self.outline = outline  # 윤곽선 목록 (mask_outlines) 또는 None
        self.conf = conf
        self.scores = {name: conf}  # 이름 -> 최근 신뢰도 누적 (STABLE_LABEL_DECAY로 감쇠)
        self.hits = 1
        self.missed = 0

    @property
    def name(self):
        return max(self.scores, key=self.scores.get)

    def update(self, name, conf, box, outline):
        a = STABLE_BOX_SMOOTH
        self.box = self.box * a + box * (1.0 - a)
        if outline is not None:
            self.outline = outline  # 이번에 윤곽이 없으면 직전 것을 그대로 (박스로 깜빡이지 않게)
        self.conf = conf
        for k in self.scores:
            self.scores[k] *= STABLE_LABEL_DECAY
        self.scores[name] = self.scores.get(name, 0.0) + conf
        self.hits += 1
        self.missed = 0


def stable_counts(detect_hz):
    """인식 빈도 (Hz) -> (나타나는 데 필요한 인식 횟수, 사라지기 전까지 놓쳐도 되는 횟수).
    STABLE_SHOW_AFTER_S / STABLE_HOLD_S를 횟수로 바꾼다. 0 (제한 없음)이면 약 25 Hz로 본다
    (1007b 화면 루프 속도). 15 Hz -> (2, 4), 25 Hz -> (3, 6, 1007b와 같음), 30 Hz -> (4, 8)."""
    hz = detect_hz if detect_hz > 0 else 25.0
    return max(STABLE_MIN_HITS_FLOOR, round(STABLE_SHOW_AFTER_S * hz)), max(1, round(STABLE_HOLD_S * hz))


class StableObjects:
    """카메라 하나의 화면 위 물체 목록 (모듈 docstring의 "화면 정리" 참고).
    min_hits: 이만큼 잡혀야 나타남, hold: 이만큼 연속으로 놓쳐야 사라짐 (둘 다 인식 횟수, stable_counts)."""

    def __init__(self, min_hits=3, hold=6):
        self.objects = []
        self.min_hits = min_hits
        self.hold = hold

    def clear(self):
        self.objects = []

    def update(self, dets):
        unmatched = list(self.objects)
        for name, conf, box, outline in sorted(dets, key=lambda d: d[1], reverse=True):
            best, best_iou = None, STABLE_MATCH_IOU
            for obj in unmatched:
                iou = box_iou(obj.box, box)
                if iou >= best_iou:
                    best, best_iou = obj, iou
            if best is None:
                self.objects.append(StableObject(name, conf, box, outline))
            else:
                best.update(name, conf, box, outline)
                unmatched.remove(best)
        for obj in unmatched:
            obj.missed += 1  # 놓친 프레임에는 직전 윤곽선을 그대로 보여줌 (박스도 직전 위치 그대로)
        self.objects = [o for o in self.objects if o.missed <= self.hold]

    def visible(self):
        shown = [o for o in self.objects if o.hits >= self.min_hits]
        shown.sort(key=lambda o: o.conf, reverse=True)
        return shown[:MAX_OBJECTS_PER_CAMERA]


PALETTE = [  # BGR
    (56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255), (49, 210, 207), (10, 249, 72),
    (23, 204, 146), (134, 219, 61), (211, 188, 0), (209, 99, 0), (255, 194, 0), (147, 69, 52),
    (255, 115, 100), (236, 24, 0), (255, 56, 132), (133, 0, 82), (255, 56, 203), (200, 149, 255),
]


def label_color(name):
    return PALETTE[zlib.crc32(name.encode()) % len(PALETTE)]  # 같은 이름은 항상 같은 색


def draw_objects(img, objects, outline_mode):
    """outline_mode = True면 물체 윤곽선 (윤곽이 없는 물체만 박스), False면 예전처럼 박스.
    이름표는 둘 다 박스 왼쪽 위 (박스는 부드럽게 따라가서 이름표가 덜 흔들린다)."""
    for o in objects:
        color = label_color(o.name)
        x1, y1, x2, y2 = (int(v) for v in o.box)
        if outline_mode and o.outline is not None:
            cv2.polylines(img, o.outline, True, color, OUTLINE_THICKNESS, cv2.LINE_AA)
        else:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        text = f"{o.name} {o.conf:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = y1 - 4 if y1 - th - 6 >= 0 else y1 + th + 4
        cv2.rectangle(img, (x1, ty - th - 2), (x1 + tw + 4, ty + 2), color, -1)
        cv2.putText(img, text, (x1 + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def list_cameras(max_index=8):
    """열리는 카메라 번호마다 해상도와 평균 밝기를 출력 (렌즈를 가려서 번호 찾기용)."""
    print("index  opened  size       brightness  (cover a lens -> its brightness drops)")
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx, BACKEND)
        if not cap.isOpened():
            cap.release()
            continue
        frame = None
        for _ in range(20):  # 노출 안정화
            ok, f = cap.read()
            if ok:
                frame = f
        if frame is None:
            print(f"{idx:>5}  yes     (no frame)")
        else:
            h, w = frame.shape[:2]
            print(f"{idx:>5}  yes     {w}x{h:<6} {frame.mean():8.1f}")
        cap.release()


def put_text(img, text, org, scale=0.6, color=(255, 255, 255)):
    """검은 테두리가 있는 글자 (밝은 배경에서도 읽히게)."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def blank_tile(text):
    tile = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), np.uint8)
    put_text(tile, text, (20, CAMERA_HEIGHT // 2))
    return tile


def parse_args():
    p = argparse.ArgumentParser(description="XLeRobot 카메라 3대 라이브 뷰 + YOLOE 물체 인식")
    p.add_argument("--list", action="store_true", help="열리는 카메라 번호와 밝기만 출력하고 끝")
    p.add_argument("--cams", default=",".join(CAMERA_ORDER), help="쓸 카메라 (예: head 또는 left,head,right)")
    p.add_argument("--head", type=int, default=CAMERA_DEFAULTS["head"], help="헤드 카메라 번호")
    p.add_argument("--left", type=int, default=CAMERA_DEFAULTS["left"], help="왼팔 손목 카메라 번호")
    p.add_argument("--right", type=int, default=CAMERA_DEFAULTS["right"], help="오른팔 손목 카메라 번호")
    p.add_argument("--no-detect", action="store_true", help="인식 없이 카메라만")
    p.add_argument("--prompt", default=",".join(PROMPT_WORDS),
                   help='찾을 물체, 쉼표로 (기본: PROMPT_WORDS 16개). 예: "cup,bottle"')
    p.add_argument("--block", default=",".join(BLOCK_WORDS),
                   help='블랙리스트, 쉼표로 (기본: BLOCK_WORDS = desk). 끄려면 --block ""')
    p.add_argument("--conf", type=float, default=DETECT_CONF, help="검출 신뢰도 하한")
    p.add_argument("--device", default=None, help="cuda:0 / cpu (기본: GPU가 있으면 GPU)")
    p.add_argument("--detect-hz", type=float, default=DETECT_HZ,
                   help=f"인식 빈도 상한 Hz (기본 {DETECT_HZ:g}). 0 = 제한 없음 (1007b처럼 화면마다)")
    args = p.parse_args()
    if not args.detect_hz >= 0.0:
        p.error("--detect-hz must be >= 0")
    args.cams = [c.strip() for c in args.cams.split(",") if c.strip()]
    bad = [c for c in args.cams if c not in CAMERA_DEFAULTS]
    if bad:
        p.error(f"unknown camera(s) {bad}; choose from {list(CAMERA_DEFAULTS)}")
    args.cams = [c for c in CAMERA_ORDER if c in args.cams]  # 화면 순서는 항상 왼 | 헤드 | 오른
    args.prompt_words = [w.strip() for w in args.prompt.split(",") if w.strip()]
    args.block_words = [w.strip() for w in args.block.split(",") if w.strip()]
    if not args.prompt_words:
        p.error("--prompt needs at least one word")
    return args


def pick_device(requested):
    if requested:
        return requested
    try:
        import torch

        if torch.cuda.is_available():
            print(f"[detect] GPU: {torch.cuda.get_device_name(0)}")
            return "cuda:0"
        print("[detect] WARNING: no CUDA GPU visible to torch -- running on CPU (slow). "
              "Check that .venv-vision has the CUDA build of torch.")
    except ImportError:
        pass
    return "cpu"


def main():
    args = parse_args()
    if args.list:
        list_cameras()
        return 0

    indices = {"head": args.head, "left": args.left, "right": args.right}
    readers = {}
    for name in args.cams:
        try:
            readers[name] = CameraReader(name, indices[name])
            print(f"[camera] {name}: index {indices[name]} opened")
        except RuntimeError as e:
            print(f"[camera] WARNING: {e} -- skipped (try --list)")
    if not readers:
        print("[camera] no camera opened -- nothing to show")
        return 1

    detector = None
    if not args.no_detect:
        device = pick_device(args.device)
        print("[detect] loading YOLOE (first run downloads the weights)...")
        detector = Detector(args.prompt_words, device, conf=args.conf, block_words=args.block_words)
        print(f"[detect] {detector.weights} ({detector.mode}) on {device}")
        print(f"[detect] words: {', '.join(detector.shown)}")

    print("Keys: q/Esc = quit, d = detection on/off, m = outline / box view, s = snapshot")
    detect_on = detector is not None
    outline_mode = True  # 1008: 기본은 윤곽선 (m으로 박스와 바꿈)
    detect_period = 1.0 / args.detect_hz if args.detect_hz > 0 else 0.0
    min_hits, hold = stable_counts(args.detect_hz)
    if detector is not None:
        limit = f"{args.detect_hz:g} Hz max" if args.detect_hz > 0 else "no limit"
        print(f"[detect] rate: {limit} (show after {min_hits} hits, drop after {hold} misses)")
    stable = {name: StableObjects(min_hits, hold) for name in readers}
    infer_ms = 0.0
    next_detect = 0.0  # 다음 인식 시각 (perf_counter)
    detect_rate = 0.0  # 실제 인식 빈도 (Hz, 약 1초마다 갱신)
    rate_n, rate_t0 = 0, time.perf_counter()
    last_ids = None  # 마지막으로 그린 카메라 프레임 번호들 (새 프레임이 없으면 다시 그리지 않음)
    redraw = True
    canvas = None
    last_print = 0.0
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            snap = {name: r.latest() for name, r in readers.items()}
            frames = {name: s[0] for name, s in snap.items()}
            ids = tuple(s[1] for s in snap.values())
            live = [name for name, f in frames.items() if f is not None]

            # 인식은 detect_period마다 한 번 (1008). 밀렸으면 몰아서 따라잡지 않고 지금부터 다시.
            now = time.perf_counter()
            if detect_on and live and now >= next_detect:
                next_detect = next_detect + detect_period if next_detect + detect_period > now else now + detect_period
                t0 = time.perf_counter()
                out = detector([frames[n] for n in live])
                infer_ms = (time.perf_counter() - t0) * 1000.0
                for name, dets in zip(live, out):
                    stable[name].update(dets)
                rate_n += 1
                redraw = True
            if now - rate_t0 >= 1.0:
                detect_rate = rate_n / (now - rate_t0)
                rate_n, rate_t0 = 0, now
            if ids != last_ids:
                redraw = True

            if redraw:
                last_ids = ids
                redraw = False
                canvas = render(args, readers, frames, stable, detector, detect_on, outline_mode, infer_ms, detect_rate)
                if canvas.shape[1] > MAX_WINDOW_WIDTH:
                    f = MAX_WINDOW_WIDTH / canvas.shape[1]
                    shown = cv2.resize(canvas, None, fx=f, fy=f, interpolation=cv2.INTER_AREA)
                else:
                    shown = canvas
                cv2.imshow(WINDOW_NAME, shown)

            now = time.monotonic()
            if detect_on and live and now - last_print >= LABEL_PRINT_INTERVAL_S:
                last_print = now
                for name in live:
                    labels = [f"{o.name} {o.conf:.2f}" for o in stable[name].visible()]
                    print(f"[{name}] " + (", ".join(labels) if labels else "-"))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("d"):
                if detector is None:
                    print("[detect] started with --no-detect -- restart without it to use detection")
                else:
                    detect_on = not detect_on
                    for s in stable.values():
                        s.clear()  # 다시 켤 때 예전 물체가 남아 있지 않게
                    next_detect = 0.0
                    redraw = True
                    print(f"[detect] {'ON' if detect_on else 'OFF'}")
            elif key == ord("m"):
                outline_mode = not outline_mode
                redraw = True
                print(f"[detect] view: {'outline' if outline_mode else 'box'}")
            elif key == ord("s") and canvas is not None:
                os.makedirs(CAPTURE_DIR, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                for name, frame in frames.items():
                    if frame is not None:
                        cv2.imwrite(os.path.join(CAPTURE_DIR, f"{stamp}_{name}.jpg"), frame)
                cv2.imwrite(os.path.join(CAPTURE_DIR, f"{stamp}_view.jpg"), canvas)
                print(f"[snapshot] saved {CAPTURE_DIR}/{stamp}_*.jpg")
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break  # 창 닫기 버튼
    except KeyboardInterrupt:
        pass
    finally:
        for r in readers.values():
            r.close()
        cv2.destroyAllWindows()
    print("Stopped.")
    return 0


def render(args, readers, frames, stable, detector, detect_on, outline_mode, infer_ms, detect_rate):
    """카메라 화면들을 이어 붙이고 물체, 카메라별 머리 줄, 아래 상태 줄을 그린 canvas를 돌려준다
    (창 크기에 맞춘 축소는 하지 않음, 스냅샷은 이 원본 크기로 저장)."""
    tiles = []
    for name in args.cams:
        frame = frames.get(name)
        if frame is None:
            tile = blank_tile(f"{name}: no frame")
        else:
            tile = frame.copy()
            header = f"{name} #{readers[name].index}  {readers[name].fps:4.1f} fps"
            if detect_on:
                objects = stable[name].visible()
                draw_objects(tile, objects, outline_mode)
                header += f"  {len(objects)} obj"
            put_text(tile, header, (10, 25))
        tiles.append(tile)
    canvas = np.hstack(tiles)
    status = f"detect {'ON' if detect_on else 'OFF'} ({detector.mode if detector else 'disabled'})"
    if detect_on:
        limit = f"/{args.detect_hz:g}" if args.detect_hz > 0 else ""
        status += f"  {detect_rate:4.1f}{limit} Hz  infer {infer_ms:5.1f} ms"
    status += f"  view {'outline' if outline_mode else 'box'}   [q quit, d detect, m outline/box, s snapshot]"
    put_text(canvas, status, (10, canvas.shape[0] - 12), scale=0.55, color=(0, 255, 255))
    return canvas


if __name__ == "__main__":
    sys.exit(main())
