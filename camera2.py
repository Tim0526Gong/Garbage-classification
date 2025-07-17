import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import cv2
import serial
import serial.tools.list_ports
import numpy as np
from ultralytics import YOLO
from tkinter import *
from PIL import Image, ImageTk
from collections import defaultdict
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import time

# === 模型初始化 ===
model = YOLO("best.pt")  # 請換成你的模型路徑

# === 自動搜尋可用串口（for macOS） ===
def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'usb' in p.device.lower():
            return p.device
    return None

port_name = find_serial_port()
try:
    ser = serial.Serial(port_name, 9600, timeout=1) if port_name else None
    if ser:
        print(f"✅ 連接到序列埠: {port_name}")
    else:
        print("⚠️ 找不到序列埠，請確認裝置連接")
except serial.SerialException:
    print(f"⚠️ 無法連接到序列埠: {port_name}")
    ser = None

# === 相機初始化 ===
cap = cv2.VideoCapture(0)

# === 儲存資料夾設定 ===
save_folder = "photo/detect_snapshots"
misclassified_folder =  "photo/misclassified"
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
latest_results = None
prev_frame_time = 0
new_frame_time = 0

# === 建立主視窗 ===
window = Tk()
window.title("智慧垃圾分類系統")
window.geometry("1500x800")
window.configure(bg="#e0e0e0")

# 使用 PanedWindow 拖曳左右區塊
main_pane = PanedWindow(window, orient=HORIZONTAL, sashrelief=SUNKEN, sashwidth=8, bg="#e0e0e0")
main_pane.pack(fill=BOTH, expand=True)

# === 左側即時畫面區 ===
video_frame = Frame(main_pane, bg="#2c3e50", relief=RAISED, borderwidth=3)
main_pane.add(video_frame, minsize=400)

video_label = Label(video_frame, bg="#2c3e50")
video_label.pack(fill=BOTH, expand=True, padx=10, pady=10)

live_label = Label(video_frame, text="LIVE", font=("Arial", 12, "bold"), fg="red", bg="#2c3e50")
live_label.place(x=10, y=10)

fps_label = Label(video_frame, text="FPS: 0", font=("Arial", 10), fg="white", bg="#2c3e50")
fps_label.place(x=10, y=35)

# === 右側：可滾動快照 + 統計圖 ===
right_outer = Frame(main_pane, bg="#ecf0f1")
main_pane.add(right_outer, minsize=500)

canvas = Canvas(right_outer, bg="#ecf0f1", highlightthickness=0)
scrollbar = Scrollbar(right_outer, orient=VERTICAL, command=canvas.yview)
canvas.configure(yscrollcommand=scrollbar.set)

scrollbar.pack(side=RIGHT, fill=Y)
canvas.pack(side=LEFT, fill=BOTH, expand=True)

right_frame = Frame(canvas, bg="#ecf0f1")
canvas.create_window((0, 0), window=right_frame, anchor='nw')

def on_frame_configure(event):
    canvas.configure(scrollregion=canvas.bbox("all"))

right_frame.bind("<Configure>", on_frame_configure)

# === 右側內容 ===
snapshot_title = Label(right_frame, text="🖼 最新快照", font=("Arial", 14, "bold"), bg="#ecf0f1")
snapshot_title.pack(anchor=W, pady=(10, 0))

snapshot_label = Label(right_frame, bg="#34495e")
snapshot_label.pack(pady=5)

conf_label = Label(right_frame, text="信心值: --%", font=("Arial", 12), bg="#ecf0f1")
conf_label.pack()

chart_title = Label(right_frame, text="📊 分類統計圖", font=("Arial", 14, "bold"), bg="#ecf0f1")
chart_title.pack(anchor=W, pady=(20, 0))

fig, ax = plt.subplots(figsize=(5, 2.5))
bar_canvas = FigureCanvasTkAgg(fig, master=right_frame)
bar_canvas.get_tk_widget().pack()

status_label = Label(right_frame, text="", font=("Arial", 12), bg="#ecf0f1")
status_label.pack(pady=(10, 0))

btn = Button(right_frame, text="📸 拍照 + 傳送", font=("Arial", 14), bg="#3498db", fg="white",
             command=lambda: capture_snapshot())
btn.pack(pady=10, ipadx=10, ipady=5)

wrong_btn = Button(right_frame, text="❌ 標記為誤判", font=("Arial", 12), bg="#e74c3c", fg="white",
                   command=lambda: save_wrong_prediction())
wrong_btn.pack(pady=5)

# === 更新統計圖 ===
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

# === 拍照傳送 ===
def capture_snapshot():
    global snapshot_count, latest_snapshot, latest_label, latest_conf, latest_results
    ret, frame = cap.read()
    if not ret:
        status_label.config(text="❌ 拍照失敗", fg="red")
        return

    results = model(frame, verbose=False)[0]
    best_label, best_conf = None, 0

    for box in results.boxes:
        cls = int(box.cls[0])
        label = model.names[cls]
        conf = float(box.conf[0])
        if label in object_to_action and conf > best_conf:
            best_label, best_conf = label, conf

    snapshot_count += 1
    fname = f"{best_label or 'unknown'}_{snapshot_count:03d}.jpg"
    cv2.imwrite(os.path.join(save_folder, fname), frame)

    if best_label:
        action = object_to_action[best_label]
        if ser and ser.is_open:
            ser.write((action + '\n').encode())
        total_counts[best_label] += 1
        latest_label, latest_conf = best_label, best_conf * 100
        status_label.config(text=f"✅ 傳送指令: {best_label}", fg="green")
    else:
        if ser and ser.is_open:
            ser.write(b'R\n')
        latest_label, latest_conf = "None", 0
        status_label.config(text="❎ 無效物件，發送重置", fg="orange")

    latest_snapshot = frame.copy()
    latest_results = results
    update_bar_chart()
    show_snapshot()

# === 顯示快照圖像 ===
def show_snapshot():
    if latest_snapshot is not None and latest_results is not None:
        snap = latest_snapshot.copy()
        for box in latest_results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = model.names[int(box.cls[0])]
            cv2.rectangle(snap, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(snap, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        img = Image.fromarray(cv2.cvtColor(snap, cv2.COLOR_BGR2RGB))
        img = img.resize((400, 300))
        snapshot_label.imgtk = ImageTk.PhotoImage(image=img)
        snapshot_label.configure(image=snapshot_label.imgtk)
        conf_label.config(text=f"類別: {latest_label}　信心值: {latest_conf:.1f}%")

# === 儲存錯誤樣本 ===
def save_wrong_prediction():
    ret, frame = cap.read()
    if not ret:
        status_label.config(text="❌ 拍照失敗", fg="red")
        return
    label_folder = os.path.join(misclassified_folder, latest_label or "unknown")
    os.makedirs(label_folder, exist_ok=True)
    path = os.path.join(label_folder, f"wrong_{latest_label}_{snapshot_count}.jpg")
    cv2.imwrite(path, frame)
    status_label.config(text=f"⚠️ 儲存錯誤樣本：{path}", fg="red")

# === 更新即時畫面 + 等比例縮放 ===
def update_video():
    global prev_frame_time
    ret, frame = cap.read()
    if ret:
        results = model(frame, verbose=False)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # FPS 顯示
        now = time.time()
        fps = 1 / (now - prev_frame_time + 1e-10)
        prev_frame_time = now
        fps_label.config(text=f"FPS: {fps:.1f}")

        # 等比例縮放
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)

        label_w, label_h = video_label.winfo_width(), video_label.winfo_height()
        label_ratio = label_w / label_h
        img_ratio = img.width / img.height
        if img_ratio > label_ratio:
            new_width = label_w
            new_height = int(label_w / img_ratio)
        else:
            new_height = label_h
            new_width = int(label_h * img_ratio)
        if new_width > 0 and new_height > 0:
            img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)

        video_label.imgtk = ImageTk.PhotoImage(image=img)
        video_label.configure(image=video_label.imgtk)

    window.after(30, update_video)

# === 關閉前釋放資源 ===
def on_close():
    cap.release()
    if ser and ser.is_open:
        ser.close()
    window.destroy()

window.protocol("WM_DELETE_WINDOW", on_close)
update_video()
window.mainloop()