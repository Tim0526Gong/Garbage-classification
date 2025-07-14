import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import cv2
import serial
import numpy as np
from ultralytics import YOLO
from tkinter import *
from PIL import Image, ImageTk
from collections import defaultdict
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import time

# === 模型與硬體初始化 ===
model = YOLO(r"C:\Users\user\Documents\pico-robotic-arm-main (2)\pico-robotic-arm-main\best.pt")

try:
    ser = serial.Serial('COM5', 9600, timeout=1)
except serial.SerialException:
    print("⚠️ 無法連接到序列埠 COM1，請確認裝置或埠號")
    ser = None

cap = cv2.VideoCapture(0)

# === 資料儲存設定 ===
save_folder = r"C:\Users\user\Documents\pwm\pwm\pwm_sg90_uart\photo\detect_snapshots"
misclassified_folder = r"C:\Users\user\Documents\pwm\pwm\pwm_sg90_uart\photo\misclassified"
os.makedirs(save_folder, exist_ok=True)
os.makedirs(misclassified_folder, exist_ok=True)

# === 分類對應指令 ===
object_to_action = {
    "plastic": "A",
    "glass": "G",
    "metal": "M",
    "paper": "P"
}

snapshot_count = 0
total_counts = defaultdict(int)
latest_snapshot = None
latest_label = ""
latest_conf = 0
latest_results = None  # 拍照時的辨識結果

# FPS 計算
prev_frame_time = 0
new_frame_time = 0

# === Tkinter 主視窗設定 ===
window = Tk()
window.title("智慧垃圾分類系統")
window.geometry("1500x750")
window.configure(bg="#e0e0e0")

main_frame = Frame(window, bg="#e0e0e0")
main_frame.pack(fill=BOTH, expand=True)

# === 左邊即時畫面區塊 ===
video_frame = Frame(main_frame, bg="#2c3e50", relief=RAISED, borderwidth=3)
video_frame.pack(side=LEFT, padx=20, pady=20)

video_label = Label(video_frame)
video_label.pack()

live_label = Label(video_frame, text="LIVE", font=("Arial", 12, "bold"), fg="red", bg="#2c3e50")
live_label.place(x=10, y=10)

fps_label = Label(video_frame, text="FPS: 0", font=("Arial", 10), fg="white", bg="#2c3e50")
fps_label.place(x=10, y=35)

# === 右側快照 + 統計圖區 ===
right_frame = Frame(main_frame, bg="#ecf0f1")
right_frame.pack(side=RIGHT, fill=BOTH, expand=True, padx=20, pady=20)

# 快照顯示
snapshot_title = Label(right_frame, text="🖼 最新快照", font=("Arial", 14, "bold"), bg="#ecf0f1")
snapshot_title.pack(anchor=W)

snapshot_label = Label(right_frame, bg="#34495e")
snapshot_label.pack(pady=5)

conf_label = Label(right_frame, text="信心值: --%", font=("Arial", 12), bg="#ecf0f1")
conf_label.pack()

# 分類統計圖標題
chart_title = Label(right_frame, text="📊 分類統計圖", font=("Arial", 14, "bold"), bg="#ecf0f1")
chart_title.pack(anchor=W, pady=(20, 0))

# matplotlib 統計圖（動態生成）
fig, ax = plt.subplots(figsize=(5, 2.5))
bar_canvas = FigureCanvasTkAgg(fig, master=right_frame)
bar_canvas.get_tk_widget().pack()

# 狀態訊息顯示
status_label = Label(right_frame, text="", font=("Arial", 12), bg="#ecf0f1")
status_label.pack(pady=(10,0))

# 按鈕
btn = Button(right_frame, text="📸 拍照 + 傳送", font=("Arial", 14), bg="#3498db", fg="white",
             command=lambda: capture_snapshot())
btn.pack(pady=10, ipadx=10, ipady=5)

wrong_btn = Button(right_frame, text="❌ 標記為誤判", font=("Arial", 12), bg="#e74c3c", fg="white",
                   command=lambda: save_wrong_prediction())
wrong_btn.pack(pady=5)

# === 畫分類統計圖表 ===
def update_bar_chart():
    ax.clear()
    labels = list(total_counts.keys())
    values = [total_counts[l] for l in labels]
    colors = ['#e74c3c', '#f39c12', '#2ecc71', '#9b59b6']
    ax.barh(labels, values, color=colors[:len(labels)])
    ax.set_title("分類統計")
    for i, v in enumerate(values):
        ax.text(v + 1, i, str(v), va='center')
    fig.tight_layout()
    bar_canvas.draw()

# === 拍照與傳送指令 ===
def capture_snapshot():
    global snapshot_count, latest_snapshot, latest_label, latest_conf, latest_results
    ret, frame = cap.read()
    if not ret:
        status_label.config(text="❌ 拍照失敗，無法取得影像", fg="red")
        window.after(2000, lambda: status_label.config(text=""))
        return
    results = model(frame, verbose=False)[0]

    best_label = None
    best_conf = 0

    for box in results.boxes:
        cls = int(box.cls[0])
        label = model.names[cls]
        conf = float(box.conf[0])
        if label in object_to_action and conf > best_conf:
            best_label = label
            best_conf = conf

    snapshot_count += 1

    if best_label:
        snapshot_path = os.path.join(save_folder, f"{best_label}_{snapshot_count:03d}.jpg")
    else:
        snapshot_path = os.path.join(save_folder, f"unknown_{snapshot_count:03d}.jpg")

    cv2.imwrite(snapshot_path, frame)

    if best_label:
        action = object_to_action[best_label]
        if ser and ser.is_open:
            ser.write((action + '\n').encode())
        total_counts[best_label] += 1
        latest_label = best_label
        latest_conf = best_conf * 100
        status_label.config(text=f"✅ 已拍攝並送出指令: {best_label}", fg="green")
        print(f"✅ 偵測到：{best_label}（{best_conf:.2f}），送出指令：{action}")
    else:
        if ser and ser.is_open:
            ser.write(b'R\n')
        latest_label = "None"
        latest_conf = 0
        status_label.config(text="❎ 未偵測到有效物件，已發送重置指令", fg="orange")
        print("❎ 無符合物件，發送 reset")

    latest_snapshot = frame.copy()
    latest_results = results
    update_bar_chart()
    show_snapshot()
    window.after(2000, lambda: status_label.config(text=""))

# === 顯示快照與類別（用拍照結果畫框，避免重複推論） ===
def show_snapshot():
    global latest_snapshot, latest_results
    if latest_snapshot is not None and latest_results is not None:
        snap = latest_snapshot.copy()
        for box in latest_results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            label = f"{model.names[cls]}"
            color = (255, 0, 0)
            cv2.rectangle(snap, (x1, y1), (x2, y2), color, 2)
            cv2.putText(snap, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        rgb = cv2.cvtColor(snap, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        img = img.resize((300, 200))
        imgtk = ImageTk.PhotoImage(image=img)
        snapshot_label.imgtk = imgtk
        snapshot_label.configure(image=imgtk)

        # 顯示「類別 + 信心值」
        conf_label.config(text=f"類別: {latest_label}　信心值: {latest_conf:.1f}%")


# === 儲存誤判影像 ===
def save_wrong_prediction():
    global snapshot_count
    ret, frame = cap.read()
    if not ret:
        status_label.config(text="❌ 拍照失敗，無法取得影像", fg="red")
        window.after(2000, lambda: status_label.config(text=""))
        return

    # 取得目前標籤，如果沒有則標為 unknown
    label_name = latest_label if latest_label else "unknown"

    # 建立對應分類資料夾（如 misclassified/plastic）
    label_folder = os.path.join(misclassified_folder, label_name)
    os.makedirs(label_folder, exist_ok=True)

    # 儲存圖片檔案
    path = os.path.join(label_folder, f"wrong_{label_name}_{snapshot_count}.jpg")
    cv2.imwrite(path, frame)

    status_label.config(text=f"⚠️ 錯誤樣本儲存至：{path}", fg="red")
    window.after(2000, lambda: status_label.config(text=""))
    print(f"⚠️ 錯誤樣本儲存至：{path}")

# === 即時畫面更新（含 FPS 顯示） ===
def update_video():
    global prev_frame_time, new_frame_time
    ret, frame = cap.read()
    if ret:
        results = model(frame, verbose=False)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            label = f"{model.names[cls]} {box.conf[0]:.2f}"
            color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # FPS 計算
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time + 1e-10)
        prev_frame_time = new_frame_time
        fps_label.config(text=f"FPS: {fps:.1f}")

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb_frame)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)

    window.after(30, update_video)

# === 關閉系統前釋放資源 ===
def on_close():
    cap.release()
    if ser and ser.is_open:
        ser.close()
    window.destroy()

window.protocol("WM_DELETE_WINDOW", on_close)
update_video()
window.mainloop()
