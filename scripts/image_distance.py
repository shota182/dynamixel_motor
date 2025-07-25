#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
o：画像を開き直す          r：描画リセット        Esc：終了
クリック 2 点 → 元画像ピクセル距離を表示（HEIC 対応）
"""
import cv2
import math
import numpy as np
from tkinter import Tk, filedialog
from PIL import Image
from pillow_heif import register_heif_opener
register_heif_opener()

# ── GUI（Tkinter）は 1 回だけ初期化 ──────────────────────────────────────────
root = Tk(); root.withdraw()
window = "Measure distance (px)"
cv2.namedWindow(window, cv2.WINDOW_NORMAL)

# ── 汎用関数：画像読み込み ──────────────────────────────────────────────────
def load_image(path: str) -> np.ndarray:
    img = cv2.imread(path)
    if img is not None:
        return img
    pil = Image.open(path).convert("RGB")
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)

# ── 汎用関数：画像選択＋セットアップ ────────────────────────────────────────
def open_new_image() -> bool:
    """新しい画像を開き、グローバル変数を再設定。成功=True／キャンセル=False"""
    global orig, disp, scale, pts_disp
    file_path = filedialog.askopenfilename(
        title="画像ファイルを選択",
        filetypes=[("Image files", "*.jpg *.jpeg *.png *.heic *.heif")],
    )
    if not file_path:
        return False                        # ユーザがキャンセル
    orig = load_image(file_path)            # 元画像
    # 画面サイズに合わせて縮小
    scr_w, scr_h = root.winfo_screenwidth(), root.winfo_screenheight()
    scale = min(scr_w / orig.shape[1], scr_h / orig.shape[0], 1.0)
    disp = (cv2.resize(orig, dsize=None, fx=scale, fy=scale,
                       interpolation=cv2.INTER_AREA)
            if scale < 1 else orig.copy())
    pts_disp.clear()
    print(f"[Opened] {file_path}")
    return True

# ── 初回画像オープン ────────────────────────────────────────────────────────
pts_disp = []               # 表示画像側のクリック座標
if not open_new_image():
    raise SystemExit("画像が選択されませんでした")

# ── マウスコールバック ─────────────────────────────────────────────────────
def mouse_cb(event, x, y, flags, userdata):
    global disp
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    pts_disp.append((x, y))
    cv2.circle(disp, (x, y), 4, (0, 255, 0), -1)
    if len(pts_disp) == 2:
        (xd1, yd1), (xd2, yd2) = pts_disp
        x1, y1 = int(xd1 / scale), int(yd1 / scale)
        x2, y2 = int(xd2 / scale), int(yd2 / scale)
        dist = math.hypot(x2 - x1, y2 - y1)
        mid = (int((xd1 + xd2) / 2), int((yd1 + yd2) / 2))
        cv2.line(disp, (xd1, yd1), (xd2, yd2), (255, 0, 0), 1)
        cv2.putText(disp, f"{dist:.1f} px", (mid[0], mid[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        print(f"Distance: {dist:.2f} pixels")

cv2.setMouseCallback(window, mouse_cb)

# ── メインループ ───────────────────────────────────────────────────────────
while True:
    cv2.imshow(window, disp)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('o'):           # 画像を開き直す
        open_new_image()
    elif key == ord('r'):         # 描画のみリセット
        disp = (cv2.resize(orig, dsize=None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
                 if scale < 1 else orig.copy())
        pts_disp.clear()
        print("[Reset]")
    elif key == 27:               # Esc → 終了
        break

cv2.destroyAllWindows()
