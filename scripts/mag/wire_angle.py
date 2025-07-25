#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
o：画像を開き直す　r：描画リセット　Esc：終了
左クリック 3 点 → 内角を計算（2 点目が頂点）
"""
import cv2
import math
import numpy as np
from tkinter import Tk, filedialog
from PIL import Image
from pillow_heif import register_heif_opener
register_heif_opener()          # HEIC 対応

# ── Tkinter 初期化 ───────────────────────────────────────────────────────────
root = Tk(); root.withdraw()
window = "Measure inner angle"
cv2.namedWindow(window, cv2.WINDOW_NORMAL)

# ── 汎用：画像読込（JPEG/PNG/HEIC 等）────────────────────────────────────────
def load_image(path: str) -> np.ndarray:
    img = cv2.imread(path)
    if img is not None:
        return img
    pil = Image.open(path).convert("RGB")
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)

# ── 汎用：画像選択＆セットアップ ────────────────────────────────────────────
def open_new_image() -> bool:
    global orig, disp, scale, pts_disp
    file = filedialog.askopenfilename(
        title="画像ファイルを選択",
        filetypes=[("Image files", "*.jpg *.jpeg *.png *.heic *.heif")],
    )
    if not file:
        return False
    orig = load_image(file)
    sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
    scale = min(sw / orig.shape[1], sh / orig.shape[0], 1.0)
    disp = (cv2.resize(orig, dsize=None, fx=scale, fy=scale,
                       interpolation=cv2.INTER_AREA)
            if scale < 1 else orig.copy())
    pts_disp.clear()
    print(f"[Opened] {file}")
    return True

# 初回
pts_disp = []        # 表示画像座標（クリック順保持）
if not open_new_image():
    raise SystemExit("画像が選択されませんでした")

# ── クリック→角度計算 ───────────────────────────────────────────────────────
def calc_inner_angle(p1, p2, p3) -> float:
    """p1,p2,p3 は元画像座標タプル (x,y)。内角(°)を返す"""
    v1 = np.array([p1[0]-p2[0], p1[1]-p2[1]], dtype=float)
    v2 = np.array([p3[0]-p2[0], p3[1]-p2[1]], dtype=float)
    # 角度 = arccos( (v1·v2) / (|v1||v2|) )
    cos_th = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_th = np.clip(cos_th, -1.0, 1.0)      # 数値誤差対策
    return math.degrees(math.acos(cos_th))

def mouse_cb(event, x, y, flags, userdata):
    global disp, pts_disp
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    pts_disp.append((x, y))
    cv2.circle(disp, (x, y), 4, (0, 255, 0), -1)

    if len(pts_disp) == 3:
        # 表示→元画像へ補正
        p1d, p2d, p3d = pts_disp
        p1 = (p1d[0] / scale, p1d[1] / scale)
        p2 = (p2d[0] / scale, p2d[1] / scale)
        p3 = (p3d[0] / scale, p3d[1] / scale)

        angle = calc_inner_angle(p1, p2, p3)
        # 線描画
        cv2.line(disp, p1d, p2d, (255, 0, 0), 1)
        cv2.line(disp, p2d, p3d, (255, 0, 0), 1)
        # テキスト表示（頂点少し上）
        cv2.putText(disp, f"{angle:.1f} deg",
                    (p2d[0], p2d[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        print(f"Inner angle: {angle:.2f} deg")

cv2.setMouseCallback(window, mouse_cb)

# ── メインループ ───────────────────────────────────────────────────────────
while True:
    cv2.imshow(window, disp)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('o'):           # 画像再選択
        open_new_image()
    elif key == ord('r'):         # 描画リセット
        disp = (cv2.resize(orig, dsize=None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
                 if scale < 1 else orig.copy())
        pts_disp.clear()
        print("[Reset]")
    elif key == 27:               # Esc
        break

cv2.destroyAllWindows()
