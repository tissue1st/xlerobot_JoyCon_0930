#!/usr/bin/env python3
"""이미지 인식 모델 비교 (2026-10-07, xlerobot_model_compare_1007.py).

같은 카메라 화면에 후보 모델을 나란히 돌려서 눈으로 비교하고, 어떤 모델을 쓸지
정하기 위한 도구다. 텔레옵이나 카메라 스크립트(xlerobot_camera_MMDD.py)는
건드리지 않는다. 블랙/화이트리스트는 보류 중이라 적용하지 않는다 (모델 그대로).

후보 (--models로 고름, 기본 전부):
  yoloe-pf    YOLOE-26s-seg 프롬프트 없음 (지금 쓰는 것). 내장 단어 4585개, 박스 + 윤곽선
  yoloe-text  YOLOE-26s-seg + --prompt 단어만. 박스 + 윤곽선
  rfdetr      RF-DETR-Seg (Roboflow). COCO 80종 고정, 박스 + 윤곽선. 단어 지정 없음
  yoloworld   YOLO-World v2 (yolov8s-worldv2) + --prompt 단어만. 박스만
  sam3        SAM 3 (Meta) + --prompt 단어만. 박스 + 윤곽선. 무거움 (한 장에 수백 ms~수 초).
              가중치 sam3.pt는 자동으로 안 받아진다. HuggingFace facebook/sam3에서 접근
              신청 후 받아서 이 폴더에 두면 켜진다. 없으면 건너뛴다.
  (OV-DEIM은 뺐다: torch 2.6 / CUDA 11.8 고정이라 RTX 50에서 안 돌고, 가중치가
   Baidu/Google Drive에만 있고, 추론 API가 없다.)

설치 (.venv-vision, 2026-10-07): 기존 + rfdetr. torch/numpy/OpenCV 버전은 그대로다.
    uv pip install --python .venv-vision/Scripts/python.exe rfdetr
가중치는 처음 실행할 때 자동으로 받는다 (sam3 빼고).

실행 (작업 폴더에서):
    .venv-vision\\Scripts\\python xlerobot_model_compare_1007.py                    # 헤드 카메라, 모델 전부
    .venv-vision\\Scripts\\python xlerobot_model_compare_1007.py --cams head,left,right   # c로 카메라 바꾸기
    .venv-vision\\Scripts\\python xlerobot_model_compare_1007.py --models yoloe-pf,rfdetr
    .venv-vision\\Scripts\\python xlerobot_model_compare_1007.py --prompt "cup,bottle,chair"
    .venv-vision\\Scripts\\python xlerobot_model_compare_1007.py --images "captures/*_head.jpg"   # 저장된 사진으로

라이브 모드: 모델마다 자기 스레드에서 가장 최근 프레임을 계속 인식한다. 그래서
타일마다 갱신 속도가 다르고, 그게 곧 그 모델의 실제 속도다 (타일 위 ms / fps).
GPU를 나눠 쓰므로 혼자 돌 때보다는 느리다. 정확한 속도는 --images 결과의 평균을 본다.

키 (영상 창이 선택된 상태에서):
    q / Esc  종료
    Space    정지 화면: 지금 프레임 하나에 모든 모델을 돌려서 같은 장면으로 비교.
             다시 누르면 라이브로
    c        다음 카메라 (--cams에 준 것 중)
    m        윤곽선 켜기 / 끄기
    s        스냅샷 (captures/compare/: 비교 화면 한 장)

--images 모드: 사진마다 모든 모델을 돌려서 captures/compare/{사진이름}_compare.jpg로
저장하고, 모델별 검출 목록과 평균 추론 시간을 터미널에 찍는다 (첫 추론은 빼고 잼).
"""

import argparse
import glob
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

# 카메라 읽기와 그리기는 camera 1006 것을 그대로 쓴다 (예전 버전 파일은 고치지 않으니 안전).
from xlerobot_camera_1006 import CameraReader, StableObjects, draw_objects, put_text

CAMERA_DEFAULTS = {"head": 0, "left": 4, "right": 2}  # camera 1007과 같은 2026-10-07 번호
TILE_W, TILE_H = 640, 480
GRID_COLS = 3
MAX_WINDOW_WIDTH = 1800
WINDOW_NAME = "model compare"
OUT_DIR = os.path.join("captures", "compare")

# 단어를 줘야 하는 모델(yoloe-text, yoloworld, sam3)이 같이 쓰는 기본 단어.
# 실험실 장면 + 집을 만한 물건. --prompt로 바꾼다.
DEFAULT_PROMPT = (
    "person,chair,desk,table,fan,lamp,monitor,laptop,keyboard,mouse,cell phone,"
    "cup,bottle,box,bag,book,scissors,pen,cable,robot arm"
)

DETECT_IMGSZ = 640
MAX_DET = 30
# 모델마다 점수 눈금이 달라서 기본 하한도 따로 둔다. --conf를 주면 전부 그 값.
DEFAULT_CONF = {"yoloe-pf": 0.4, "yoloe-text": 0.3, "rfdetr": 0.5, "yoloworld": 0.3, "sam3": 0.5}
MODEL_ORDER = ("yoloe-pf", "yoloe-text", "rfdetr", "yoloworld", "sam3")
RFDETR_SIZES = ("nano", "small", "medium", "large", "xlarge", "2xlarge")
SAM3_WEIGHTS = "sam3.pt"
WARMUP_RUNS = 3


class Det:
    """화면에 그릴 물체 하나 (draw_objects가 읽는 모양: name, conf, box, poly)."""

    __slots__ = ("name", "conf", "box", "poly")

    def __init__(self, name, conf, box, poly):
        self.name, self.conf, self.box, self.poly = name, conf, box, poly


def dets_from_ultralytics(result):
    """ultralytics Results -> [(이름, 신뢰도, 박스 xyxy, 윤곽선 폴리곤 또는 None)]."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []
    xyxy = boxes.xyxy.cpu().numpy()
    polys = result.masks.xy if result.masks is not None else [None] * len(xyxy)
    return [
        (result.names[int(c)], p, box, poly if poly is not None and len(poly) >= 3 else None)
        for c, p, box, poly in zip(boxes.cls.tolist(), boxes.conf.tolist(), xyxy, polys)
    ]


def mask_to_poly(mask):
    """이진 마스크 (H, W) -> 가장 큰 윤곽선 폴리곤 (N, 2) 또는 None."""
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    return c.reshape(-1, 2).astype(np.float32) if len(c) >= 3 else None


# --- 모델 불러오기: 각 함수는 (설명, run) 을 돌려준다. run(BGR 프레임) -> dets ---------


def load_yoloe_pf(device, words, conf, args):
    from ultralytics import YOLOE

    model = YOLOE("yoloe-26s-seg-pf.pt")

    def run(frame):
        r = model.predict(frame, conf=conf, imgsz=DETECT_IMGSZ, device=device, max_det=MAX_DET,
                          agnostic_nms=True, verbose=False)[0]
        return dets_from_ultralytics(r)

    return "YOLOE-26s prompt-free (now)", run


def load_yoloe_text(device, words, conf, args):
    from ultralytics import YOLOE

    model = YOLOE("yoloe-26s-seg.pt")
    model.set_classes(words)

    def run(frame):
        r = model.predict(frame, conf=conf, imgsz=DETECT_IMGSZ, device=device, max_det=MAX_DET,
                          agnostic_nms=True, verbose=False)[0]
        return dets_from_ultralytics(r)

    return f"YOLOE-26s text ({len(words)} words)", run


def load_rfdetr(device, words, conf, args):
    import rfdetr

    cls_name = "RFDETRSeg" + {"nano": "Nano", "small": "Small", "medium": "Medium", "large": "Large",
                              "xlarge": "XLarge", "2xlarge": "2XLarge"}[args.rfdetr_size]
    model = getattr(rfdetr, cls_name)(device=device.replace("cuda:0", "cuda"))
    note = ""
    if device != "cpu":
        try:  # FP16 + 컴파일. 안 하면 rfdetr가 "not optimized" 경고를 내고 느리다
            import torch

            model.inference(dtype=torch.float16)
            note = ", fp16"
        except Exception as e:
            print(f"[rfdetr] WARNING: fp16 optimize failed ({type(e).__name__}: {e}) -- running unoptimized")

    def run(frame):
        d = model.predict(frame[:, :, ::-1].copy(), threshold=conf, include_source_image=False)  # RGB
        # class_id는 COCO 번호(1~90, 중간에 빈 번호)라서 rfdetr가 붙여 주는 이름을 쓴다
        names = d.data.get("class_name", [str(c) for c in d.class_id])
        masks = d.mask if d.mask is not None else [None] * len(d)
        out = [(str(n), float(p), box, mask_to_poly(m) if m is not None else None)
               for box, p, n, m in zip(d.xyxy, d.confidence, names, masks)]
        return sorted(out, key=lambda x: x[1], reverse=True)[:MAX_DET]

    return f"RF-DETR-Seg {args.rfdetr_size} (COCO 80{note})", run


def load_yoloworld(device, words, conf, args):
    from ultralytics import YOLOWorld

    model = YOLOWorld("yolov8s-worldv2.pt")
    model.set_classes(words)

    def run(frame):
        r = model.predict(frame, conf=conf, imgsz=DETECT_IMGSZ, device=device, max_det=MAX_DET,
                          agnostic_nms=True, verbose=False)[0]
        return dets_from_ultralytics(r)

    return f"YOLO-World v2 s ({len(words)} words, box only)", run


def load_sam3(device, words, conf, args):
    if not os.path.exists(SAM3_WEIGHTS):
        raise FileNotFoundError(
            f"{SAM3_WEIGHTS} not found -- request access at huggingface.co/facebook/sam3, "
            f"download it into this folder (about 3.4 GB)")
    from ultralytics.models.sam import SAM3SemanticPredictor

    predictor = SAM3SemanticPredictor(overrides=dict(
        model=SAM3_WEIGHTS, conf=conf, device=device, half=device != "cpu", verbose=False))

    def run(frame):
        return dets_from_ultralytics(predictor(source=frame, text=words)[0])

    return f"SAM 3 ({len(words)} words, heavy)", run


LOADERS = {"yoloe-pf": load_yoloe_pf, "yoloe-text": load_yoloe_text, "rfdetr": load_rfdetr,
           "yoloworld": load_yoloworld, "sam3": load_sam3}


class Model:
    """불러온 모델 하나 + 마지막 결과."""

    def __init__(self, key, label, run, stable):
        self.key, self.label, self.run = key, label, run
        self.stable = StableObjects() if stable else None
        self.lock = threading.Lock()
        self.frame, self.dets, self.ms, self.fps = None, [], 0.0, 0.0
        self.error = None

    def infer(self, frame):
        """frame 하나를 인식해서 결과를 저장하고 걸린 ms를 돌려준다."""
        t0 = time.perf_counter()
        raw = self.run(frame)
        ms = (time.perf_counter() - t0) * 1000.0
        if self.stable is not None:
            self.stable.update(raw)
            dets = self.stable.visible()
        else:
            dets = [Det(*d) for d in sorted(raw, key=lambda d: d[1], reverse=True)]
        with self.lock:
            self.frame, self.dets, self.ms = frame, dets, ms
        return ms

    def snapshot(self):
        with self.lock:
            return self.frame, self.dets, self.ms, self.fps


class Worker(threading.Thread):
    """라이브 모드: 모델 하나를 자기 스레드에서 source()의 최신 프레임으로 계속 돌린다."""

    def __init__(self, model, source):
        super().__init__(name=f"model-{model.key}", daemon=True)
        self.model, self.source = model, source
        self.running = True

    def run(self):
        last_id, n, t0 = None, 0, time.monotonic()
        while self.running:
            frame, fid = self.source()
            if frame is None or fid == last_id:
                time.sleep(0.003)
                continue
            last_id = fid
            try:
                self.model.infer(frame)
            except Exception as e:  # 한 모델이 죽어도 나머지는 계속
                self.model.error = f"{type(e).__name__}: {e}"
                print(f"[{self.model.key}] ERROR {self.model.error}")
                return
            n += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.model.fps = n / (now - t0)
                n, t0 = 0, now


def make_tile(model, fallback_frame, show_masks, live):
    frame, dets, ms, fps = model.snapshot()
    if frame is None:
        frame = fallback_frame
    tile = np.zeros((TILE_H, TILE_W, 3), np.uint8) if frame is None else frame.copy()
    draw_objects(tile, dets, show_masks)
    if tile.shape[:2] != (TILE_H, TILE_W):
        tile = cv2.resize(tile, (TILE_W, TILE_H))  # 사진 크기가 다르면 그린 뒤에 맞춤
    put_text(tile, model.label, (10, 25), scale=0.6, color=(0, 255, 255))
    info = f"{ms:6.1f} ms  {len(dets)} obj" + (f"  {fps:4.1f} fps" if live else "")
    put_text(tile, info, (10, 50), scale=0.55)
    if model.error:
        put_text(tile, "ERROR (see terminal)", (10, 80), color=(0, 0, 255))
    return tile


def make_grid(tiles):
    cols = min(GRID_COLS, len(tiles))
    rows = (len(tiles) + cols - 1) // cols
    tiles = tiles + [np.zeros_like(tiles[0])] * (rows * cols - len(tiles))
    return np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows)])


def fit_width(img):
    if img.shape[1] <= MAX_WINDOW_WIDTH:
        return img
    f = MAX_WINDOW_WIDTH / img.shape[1]
    return cv2.resize(img, None, fx=f, fy=f, interpolation=cv2.INTER_AREA)


def load_models(args, device):
    models = []
    for key in args.models:
        conf = args.conf if args.conf is not None else DEFAULT_CONF[key]
        print(f"[{key}] loading (first run downloads the weights)...")
        try:
            label, run = LOADERS[key](device, args.prompt_words, conf, args)
        except Exception as e:
            print(f"[{key}] SKIPPED: {type(e).__name__}: {e}")
            continue
        label += f"  conf>={conf}"
        m = Model(key, label, run, args.stable)
        t0 = time.perf_counter()
        try:
            # 처음 몇 번은 GPU 준비로 느려서 미리 돌린다 (검은 화면은 검출이 없어서 덜 데워짐 -> 잡음 화면)
            noise = np.random.default_rng(0).integers(0, 256, (TILE_H, TILE_W, 3), dtype=np.uint8)
            for _ in range(WARMUP_RUNS):
                m.run(noise)
        except Exception as e:
            print(f"[{key}] SKIPPED (warm-up failed): {type(e).__name__}: {e}")
            continue
        print(f"[{key}] ready: {label} (warm-up {time.perf_counter() - t0:.1f} s)")
        models.append(m)
    return models


def run_images(args, models):
    files = sorted(f for pat in args.images for f in glob.glob(pat))
    if not files:
        print(f"[images] no file matches {args.images}")
        return 1
    os.makedirs(OUT_DIR, exist_ok=True)
    times = {m.key: [] for m in models}
    for m in models:
        m.stable = None  # 사진끼리는 이어지지 않아서 --stable은 안 씀 (3프레임 잡혀야 보이므로)
    for path in files:
        frame = cv2.imread(path)
        if frame is None:
            print(f"[images] cannot read {path}")
            continue
        print(f"\n{path}")
        for m in models:
            times[m.key].append(m.infer(frame))
            _, dets, ms, _ = m.snapshot()
            print(f"  {m.key:10s} {ms:7.1f} ms  " + (", ".join(f"{d.name} {d.conf:.2f}" for d in dets) or "-"))
        grid = make_grid([make_tile(m, frame, args.masks, live=False) for m in models])
        out = os.path.join(OUT_DIR, os.path.splitext(os.path.basename(path))[0] + "_compare.jpg")
        cv2.imwrite(out, grid)
    print("\naverage inference time (one image at a time):")
    for m in models:
        t = times[m.key]
        if t:
            print(f"  {m.key:10s} {np.mean(t):7.1f} ms   ({m.label})")
    print(f"\nsaved {len(files)} comparison images to {OUT_DIR}/")
    return 0


def run_live(args, models):
    readers = {}
    for name in args.cams:
        try:
            readers[name] = CameraReader(name, args.indices[name])
            print(f"[camera] {name}: index {args.indices[name]} opened")
        except RuntimeError as e:
            print(f"[camera] WARNING: {e} -- skipped")
    if not readers:
        print("[camera] no camera opened -- try --images, or check indices with xlerobot_camera_1007.py --list")
        return 1
    names = list(readers)
    state = {"cam": names[0], "still": None, "still_id": 0}

    def source():
        """(프레임, 번호). 정지 화면이면 같은 프레임과 같은 번호 -> 모델마다 딱 한 번 돈다."""
        if state["still"] is not None:
            return state["still"], ("still", state["still_id"])
        frame, fid = readers[state["cam"]].latest()
        return frame, (state["cam"], fid)

    workers = [Worker(m, source) for m in models]
    for w in workers:
        w.start()
    show_masks = args.masks
    print("Keys: q/Esc quit, Space still frame on/off, c next camera, m outlines, s snapshot")
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    try:
        while True:
            live_frame = state["still"] if state["still"] is not None else readers[state["cam"]].latest()[0]
            grid = make_grid([make_tile(m, live_frame, show_masks, live=state["still"] is None) for m in models])
            mode = "STILL (Space = live)" if state["still"] is not None else "LIVE (Space = still)"
            put_text(grid, f"camera {state['cam']}  {mode}   [q quit, c camera, m outlines, s snapshot]",
                     (10, grid.shape[0] - 12), scale=0.6, color=(0, 255, 255))
            cv2.imshow(WINDOW_NAME, fit_width(grid))

            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                if state["still"] is None:
                    frame = readers[state["cam"]].latest()[0]
                    if frame is not None:
                        state["still"], state["still_id"] = frame.copy(), state["still_id"] + 1
                        print("[still] all models on the same frame")
                else:
                    state["still"] = None
                    print("[still] back to live")
            elif key == ord("c") and len(names) > 1:
                state["cam"] = names[(names.index(state["cam"]) + 1) % len(names)]
                for m in models:
                    if m.stable is not None:
                        m.stable.clear()
                print(f"[camera] {state['cam']}")
            elif key == ord("m"):
                show_masks = not show_masks
            elif key == ord("s"):
                os.makedirs(OUT_DIR, exist_ok=True)
                out = os.path.join(OUT_DIR, datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{state['cam']}.jpg")
                cv2.imwrite(out, grid)
                print(f"[snapshot] {out}")
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        for w in workers:
            w.running = False
        for w in workers:
            w.join(timeout=5.0)  # SAM 3는 한 번 도는 데 오래 걸릴 수 있음
        for r in readers.values():
            r.close()
        cv2.destroyAllWindows()
    return 0


def parse_args():
    p = argparse.ArgumentParser(description="이미지 인식 모델 비교 (같은 화면에 나란히)")
    p.add_argument("--models", default=",".join(MODEL_ORDER), help=f"비교할 모델, 쉼표로 ({', '.join(MODEL_ORDER)})")
    p.add_argument("--prompt", default=DEFAULT_PROMPT, help="단어를 줘야 하는 모델의 단어, 쉼표로")
    p.add_argument("--cams", default="head", help="쓸 카메라 (예: head 또는 head,left,right). c로 바꿈")
    p.add_argument("--head", type=int, default=CAMERA_DEFAULTS["head"])
    p.add_argument("--left", type=int, default=CAMERA_DEFAULTS["left"])
    p.add_argument("--right", type=int, default=CAMERA_DEFAULTS["right"])
    p.add_argument("--images", nargs="+", default=None, help='카메라 대신 사진 (예: "captures/*_head.jpg")')
    p.add_argument("--conf", type=float, default=None, help="모든 모델의 신뢰도 하한 (기본: 모델마다 DEFAULT_CONF)")
    p.add_argument("--rfdetr-size", default="small", choices=RFDETR_SIZES)
    p.add_argument("--stable", action="store_true", help="camera 1006의 화면 정리(StableObjects)를 모델마다 적용")
    p.add_argument("--masks", action="store_true", help="윤곽선을 켠 채로 시작")
    p.add_argument("--device", default=None, help="cuda:0 / cpu (기본: GPU가 있으면 GPU)")
    args = p.parse_args()
    args.models = [m.strip() for m in args.models.split(",") if m.strip()]
    bad = [m for m in args.models if m not in LOADERS]
    if bad:
        p.error(f"unknown model(s) {bad}; choose from {list(MODEL_ORDER)}")
    args.cams = [c.strip() for c in args.cams.split(",") if c.strip()]
    bad = [c for c in args.cams if c not in CAMERA_DEFAULTS]
    if bad:
        p.error(f"unknown camera(s) {bad}; choose from {list(CAMERA_DEFAULTS)}")
    args.indices = {"head": args.head, "left": args.left, "right": args.right}
    args.prompt_words = [w.strip() for w in args.prompt.split(",") if w.strip()]
    return args


def pick_device(requested):
    if requested:
        return requested
    import torch

    if torch.cuda.is_available():
        print(f"[device] GPU: {torch.cuda.get_device_name(0)}")
        return "cuda:0"
    print("[device] WARNING: no CUDA GPU visible to torch -- running on CPU (slow)")
    return "cpu"


def main():
    args = parse_args()
    device = pick_device(args.device)
    models = load_models(args, device)
    if not models:
        print("no model loaded")
        return 1
    return run_images(args, models) if args.images else run_live(args, models)


if __name__ == "__main__":
    sys.exit(main())
