import sensor, image, time, pyb

# ================= UART to Arduino =================
uart = pyb.UART(3, 115200, timeout_char=20)  # 不通就改 1/2/3 试

# ================= Camera Init =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240
sensor.skip_frames(time=2000)

# 室内追踪：关掉自动项，颜色稳定
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
# 如果你环境光不怎么变，也可以锁曝光（可选）
# sensor.set_auto_exposure(False, exposure_us=15000)

clock = time.clock()

W, H = 320, 240
CX0, CY0 = W // 2, H // 2

# ============ Red Threshold (LAB) ============
# 这是“红纸起步值”，红色金属通常需要更严格的 A/B 并限制亮度 L_max
# 用 Threshold Editor 调到最稳后替换这里：
RED = (25, 95, 35, 127, -20, 80)
# (Lmin, Lmax, Amin, Amax, Bmin, Bmax)

# ============ Blob / ROI Params ============
PIX_TH = 600     # 反光会碎，太小容易噪声；太大又丢目标。先250~600
AREA_TH = 600
MERGE = True

# ROI 跟随（默认全画面）
roi = (0, 0, W, H)
ROI_PAD = 80     # 跟随窗口边界扩展，越大越不容易丢，越小越快
MIN_ROI = 140    # ROI 最小宽高

# ============ Smoothing / Lost Handling ============
alpha = 0.35     # err 平滑系数（0.2~0.5）
err_f = 0

lost_cnt = 0
LOST_SEND = 3    # 连续丢失几帧才发 L（防止闪一下就进入搜索）

# 上一次目标中心（用于“离上一帧最近”选择 blob）
last_cx = CX0
last_cy = CY0
has_last = False

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def make_roi(cx, cy):
    # 以目标中心生成 ROI
    x = clamp(cx - ROI_PAD, 0, W - 1)
    y = clamp(cy - ROI_PAD, 0, H - 1)
    x2 = clamp(cx + ROI_PAD, 0, W)
    y2 = clamp(cy + ROI_PAD, 0, H)
    w = max(MIN_ROI, x2 - x)
    h = max(MIN_ROI, y2 - y)
    # 调整 w/h 不越界
    if x + w > W: x = W - w
    if y + h > H: y = H - h
    return (int(x), int(y), int(w), int(h))

while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs([RED],
                           roi=roi,
                           pixels_threshold=PIX_TH,
                           area_threshold=AREA_TH,
                           merge=MERGE)

    if blobs:
        # ========= 选 blob：面积大 + 离上一帧近（防跳） =========
        best = None
        bestScore = -1e18
        for b in blobs:
            a = b.pixels()
            cx = b.cx()
            cy = b.cy()

            if has_last:
                dx = cx - last_cx
                dy = cy - last_cy
                dist2 = dx*dx + dy*dy
            else:
                dx = cx - CX0
                dy = cy - CY0
                dist2 = dx*dx + dy*dy

            # 评分：面积越大越好，离上次越近越好
            # 反光导致碎块时，最近原则会更稳
            score = a - 0.6 * dist2
            if score > bestScore:
                bestScore = score
                best = b

        b = best
        cx, cy = b.cx(), b.cy()
        area = b.pixels()

        # 更新 last
        last_cx, last_cy = cx, cy
        has_last = True
        lost_cnt = 0

        # err（像素）
        err = cx - CX0
        err_f = int(alpha * err + (1 - alpha) * err_f)

        # ROI 跟随（提高帧率 + 抗干扰）
        roi = make_roi(cx, cy)

        # 调试绘制（跑车时可注释掉提高 fps）
        img.draw_rectangle(b.rect(), thickness=2)
        img.draw_cross(cx, cy, size=10)
        img.draw_rectangle(roi, color=(0,255,0), thickness=1)
        img.draw_string(2, 2, "err:%d area:%d fps:%d" % (err_f, area, int(clock.fps())),
                        color=(255,255,255))

        # 发给 Arduino
        uart.write("E {} {}\n".format(err_f, area))

    else:
        lost_cnt += 1
        if lost_cnt >= LOST_SEND:
            has_last = False
            roi = (0, 0, W, H)  # 丢失后回到全画面找
            uart.write("L\n")
        # 丢失前几帧不发 L，防止你 Arduino 一闪就进入搜索
