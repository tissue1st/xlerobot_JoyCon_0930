#!/usr/bin/env python3
"""XLeRobot 카메라 3대 (헤드 + 양 손목) 라이브 뷰 + YOLOE 물체 인식.

2026-10-07 두 번째 버전 (xlerobot_camera_1007b.py). 텔레옵 스크립트
(xlerobot_JoyCon_1006.py)와 따로 돌아가는 카메라 전용 테스트 스크립트다.
로봇 버스나 Joy-Con은 건드리지 않는다. 검증이 끝나면 텔레옵과 합친다.

환경: 텔레옵의 .venv와 별도인 .venv-vision (torch CUDA 빌드 + ultralytics).
텔레옵 .venv의 torch는 CPU 전용이고 lerobot이 버전을 묶고 있어서 나눴다.
RTX 50 시리즈는 CUDA 12.8 이상 빌드가 필요하다.

    uv venv .venv-vision --python 3.12
    uv pip install --python .venv-vision/Scripts/python.exe torch torchvision --index-url https://download.pytorch.org/whl/cu128
    uv pip install --python .venv-vision/Scripts/python.exe ultralytics opencv-python

실행 (작업 레포 폴더에서):

    .venv-vision\\Scripts\\python xlerobot_camera_1007b.py --list          # 열리는 카메라 번호와 밝기만
    .venv-vision\\Scripts\\python xlerobot_camera_1007b.py                 # 3대 + 인식 (PROMPT_WORDS 15개)
    .venv-vision\\Scripts\\python xlerobot_camera_1007b.py --prompt "cup,bottle,phone"   # 찾을 단어 바꾸기
    .venv-vision\\Scripts\\python xlerobot_camera_1007b.py --block "desk,chair,box"      # 블랙리스트 바꾸기
    .venv-vision\\Scripts\\python xlerobot_camera_1007b.py --block ""                    # 블랙리스트 끄기
    .venv-vision\\Scripts\\python xlerobot_camera_1007b.py --no-detect     # 카메라만 (인식 끔)
    .venv-vision\\Scripts\\python xlerobot_camera_1007b.py --cams head     # 헤드만
    .venv-vision\\Scripts\\python xlerobot_camera_1007b.py --head 0 --left 4 --right 2   # 번호 지정

키 (영상 창이 선택된 상태에서):
    q / Esc  종료
    d        인식 켜기 / 끄기
    m        윤곽선 (마스크) 켜기 / 끄기
    s        스냅샷 저장 (captures/ 폴더: 카메라별 원본 + 표시 화면)

카메라 번호 (같은 모델 USB2.0_CAM1 세 대, 640x480 YUY2 약 30fps):
    2026-10-07 실측 (기본값): 헤드 = 0, 왼팔 손목 = 4, 오른팔 손목 = 2 (처음 2/4로 적었다가 사용자 확인으로 좌우 바꿈).
    2026-10-06 실측: 헤드 = 4, 왼팔 손목 = 2, 오른팔 손목 = 3, 0 = 노트북 내장, 1 = 가상 카메라.
Windows는 꽂는 순서나 재부팅에 따라 번호를 다시 매긴다. 세 대가 같은 모델이라
이름으로는 구분이 안 된다. 화면이 엉뚱하면 --list로 번호를 보고 (렌즈 하나를
손으로 가리면 그 번호의 밝기가 뚝 떨어진다) --head/--left/--right로 지정한다.

인식 모델: YOLOE-26s-seg (ultralytics), 단어 지정 모드 (YOLOE_WEIGHTS + 텍스트 임베딩).
박스와 윤곽선이 한 번에 나온다. 가중치는 처음 실행할 때 자동으로 내려받는다.

1007b에서 바뀐 것 (2026-10-07, 모델 비교 후 사용자 결정):
  - 프롬프트 없는 모드(내장 단어 4585개)를 버리고 단어 지정 모드만 쓴다.
    xlerobot_model_compare_1007.py로 오늘 스냅샷 9장을 비교했을 때, 단어
    지정 모드는 벽 오인식이 없었고 의자, 선풍기, 램프는 그대로 잡았고 더
    빨랐다 (평균 18 ms vs 28 ms). 프롬프트 없는 모드가 필요하면 1007 / 1006 파일.
  - 찾을 단어 PROMPT_WORDS 15개 (--prompt로 바꿈).
  - 블랙리스트 BLOCK_WORDS (지금 desk, chair, --block으로 바꿈): 모델에는 단어로
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
    STABLE_MIN_HITS 프레임 이상 잡힌 물체만 보여주고, 한두 프레임 놓쳐도
    STABLE_HOLD_FRAMES 동안은 유지한다. 이름은 최근에 가장 자주/강하게
    붙은 것으로 고정해서 chair <-> office chair 처럼 바뀌는 걸 막는다.
  - 카메라당 MAX_OBJECTS_PER_CAMERA 개까지만 표시.
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
# 찾을 단어 15개 (1007b, 모델 비교 때 20개에서 줄임: table, cable, robot arm 빼고
# chair, desk는 블랙리스트로). 소문자 영어. --prompt로 바꾼다.
PROMPT_WORDS = (
    "person", "cup", "bottle", "cell phone", "box", "bag", "book", "scissors", "pen",
    "laptop", "keyboard", "mouse", "monitor", "fan", "lamp",
)
# 블랙리스트: 모델에는 단어로 넣지만 화면에는 안 보이는 것 (docstring "1007b" 참고).
# --block으로 바꾼다.
BLOCK_WORDS = ("desk", "chair")
DETECT_CONF = 0.3  # 이보다 낮은 신뢰도의 검출은 버림 (1006: 0.25 -> 0.4, 1007b: 단어 지정 모드라 0.3)
DETECT_IMGSZ = 640
DETECT_MAX_DET = 30  # 모델이 한 프레임에서 내놓는 최대 개수 (표시 전, 필터 여유분)

# 화면 정리 (모듈 docstring 참고). 프레임 수는 인식 속도 기준 (약 25fps -> 1프레임 = 40ms).
STABLE_MIN_HITS = 3  # 이만큼 잡혀야 화면에 나타남 (약 0.1초)
STABLE_HOLD_FRAMES = 6  # 이만큼 연속으로 놓쳐야 화면에서 사라짐 (약 0.25초)
STABLE_MATCH_IOU = 0.3  # 이전 물체와 박스가 이만큼 겹치면 같은 물체로 봄
STABLE_BOX_SMOOTH = 0.5  # 박스 위치 = 이전 * 이 값 + 새 값 * (1 - 이 값). 떨림 감소
STABLE_LABEL_DECAY = 0.9  # 이름 점수가 프레임마다 이만큼 줄어듦 (오래된 이름은 잊힘)
MAX_OBJECTS_PER_CAMERA = 10

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
        """프레임 리스트 -> 프레임마다 [(이름, 신뢰도, 박스 xyxy, 윤곽선 폴리곤 또는 None), ...]."""
        results = self.model.predict(
            frames, conf=self.conf, imgsz=self.imgsz, device=self.device, max_det=DETECT_MAX_DET,
            agnostic_nms=True, verbose=False,
        )
        return [extract_detections(r, self.hidden) for r in results]


def extract_detections(result, hidden=()):
    """ultralytics Results -> [(이름, 신뢰도, 박스 xyxy (np), 윤곽선 폴리곤 (np) 또는 None)].
    hidden(블랙리스트)에 있는 이름은 버린다. NMS가 끝난 뒤라서, 그 박스가 겹친
    다른 단어의 박스를 이미 지운 상태다 (의자를 의자로 잡고 통째로 숨김)."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []
    names = result.names
    xyxy = boxes.xyxy.cpu().numpy()
    polys = result.masks.xy if result.masks is not None else [None] * len(xyxy)
    dets = []
    for c, p, box, poly in zip(boxes.cls.tolist(), boxes.conf.tolist(), xyxy, polys):
        name = names[int(c)]
        if name.lower() in hidden:
            continue
        dets.append((name, p, box, poly if poly is not None and len(poly) >= 3 else None))
    return dets


def box_iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


class StableObject:
    """화면 위 물체 하나. 프레임마다의 검출을 이어 붙인 것."""

    def __init__(self, name, conf, box, poly):
        self.box = box.astype(float)
        self.poly = poly
        self.conf = conf
        self.scores = {name: conf}  # 이름 -> 최근 신뢰도 누적 (STABLE_LABEL_DECAY로 감쇠)
        self.hits = 1
        self.missed = 0

    @property
    def name(self):
        return max(self.scores, key=self.scores.get)

    def update(self, name, conf, box, poly):
        a = STABLE_BOX_SMOOTH
        self.box = self.box * a + box * (1.0 - a)
        self.poly = poly
        self.conf = conf
        for k in self.scores:
            self.scores[k] *= STABLE_LABEL_DECAY
        self.scores[name] = self.scores.get(name, 0.0) + conf
        self.hits += 1
        self.missed = 0


class StableObjects:
    """카메라 하나의 화면 위 물체 목록 (모듈 docstring의 "화면 정리" 참고)."""

    def __init__(self):
        self.objects = []

    def clear(self):
        self.objects = []

    def update(self, dets):
        unmatched = list(self.objects)
        for name, conf, box, poly in sorted(dets, key=lambda d: d[1], reverse=True):
            best, best_iou = None, STABLE_MATCH_IOU
            for obj in unmatched:
                iou = box_iou(obj.box, box)
                if iou >= best_iou:
                    best, best_iou = obj, iou
            if best is None:
                self.objects.append(StableObject(name, conf, box, poly))
            else:
                best.update(name, conf, box, poly)
                unmatched.remove(best)
        for obj in unmatched:
            obj.missed += 1
            obj.poly = None  # 놓친 프레임에는 윤곽선을 그리지 않음 (박스만 유지)
        self.objects = [o for o in self.objects if o.missed <= STABLE_HOLD_FRAMES]

    def visible(self):
        shown = [o for o in self.objects if o.hits >= STABLE_MIN_HITS]
        shown.sort(key=lambda o: o.conf, reverse=True)
        return shown[:MAX_OBJECTS_PER_CAMERA]


PALETTE = [  # BGR
    (56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255), (49, 210, 207), (10, 249, 72),
    (23, 204, 146), (134, 219, 61), (211, 188, 0), (209, 99, 0), (255, 194, 0), (147, 69, 52),
    (255, 115, 100), (236, 24, 0), (255, 56, 132), (133, 0, 82), (255, 56, 203), (200, 149, 255),
]


def label_color(name):
    return PALETTE[zlib.crc32(name.encode()) % len(PALETTE)]  # 같은 이름은 항상 같은 색


def draw_objects(img, objects, show_masks):
    if show_masks:
        overlay = img.copy()
        for o in objects:
            if o.poly is not None:
                cv2.fillPoly(overlay, [o.poly.astype(np.int32)], label_color(o.name))
        cv2.addWeighted(overlay, 0.4, img, 0.6, 0, dst=img)
    for o in objects:
        color = label_color(o.name)
        x1, y1, x2, y2 = (int(v) for v in o.box)
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
                   help='찾을 물체, 쉼표로 (기본: PROMPT_WORDS 15개). 예: "cup,bottle"')
    p.add_argument("--block", default=",".join(BLOCK_WORDS),
                   help='블랙리스트, 쉼표로 (기본: BLOCK_WORDS = desk,chair). 끄려면 --block ""')
    p.add_argument("--conf", type=float, default=DETECT_CONF, help="검출 신뢰도 하한")
    p.add_argument("--device", default=None, help="cuda:0 / cpu (기본: GPU가 있으면 GPU)")
    args = p.parse_args()
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

    print("Keys: q/Esc = quit, d = detection on/off, m = outlines (masks) on/off, s = snapshot")
    detect_on = detector is not None
    show_masks = False
    stable = {name: StableObjects() for name in readers}
    infer_ms = 0.0
    last_print = 0.0
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            frames = {name: r.latest()[0] for name, r in readers.items()}
            live = [name for name, f in frames.items() if f is not None]

            if detect_on and live:
                t0 = time.perf_counter()
                out = detector([frames[n] for n in live])
                infer_ms = (time.perf_counter() - t0) * 1000.0
                for name, dets in zip(live, out):
                    stable[name].update(dets)

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
                        draw_objects(tile, objects, show_masks)
                        header += f"  {len(objects)} obj"
                    put_text(tile, header, (10, 25))
                tiles.append(tile)
            canvas = np.hstack(tiles)
            status = f"detect {'ON' if detect_on else 'OFF'} ({detector.mode if detector else 'disabled'})"
            if detect_on:
                status += f"  infer {infer_ms:5.1f} ms"
            status += f"  outlines {'ON' if show_masks else 'OFF'}   [q quit, d detect, m outlines, s snapshot]"
            put_text(canvas, status, (10, canvas.shape[0] - 12), scale=0.55, color=(0, 255, 255))
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
                    print(f"[detect] {'ON' if detect_on else 'OFF'}")
            elif key == ord("m"):
                show_masks = not show_masks
                print(f"[detect] outlines {'ON' if show_masks else 'OFF'}")
            elif key == ord("s"):
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


if __name__ == "__main__":
    sys.exit(main())
