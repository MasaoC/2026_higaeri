from __future__ import annotations

import argparse
import queue
import re
import threading
import time
import tkinter as tk

import serial


AIR_PATTERN = re.compile(r"air=.*?spd=([0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)
WIND_PATTERN = re.compile(r"wind=.*?spd=([0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)
WIND1_PATTERN = re.compile(r"w1=([0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)
WIND2_PATTERN = re.compile(r"w2=([0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)
DISP_PATTERN = re.compile(r"disp=.*?pot1=([0-9]+)\s+pot2=([0-9]+)", re.IGNORECASE)
STALE_TIMEOUT_SEC = 2.0

# ─────────────────────────────────────────────────────────────────────────────
# 操舵角（ポテンショメータ）の構成パラメーター
# ユーザーが簡単に調整できるようにグローバル定数として配置します。
# ─────────────────────────────────────────────────────────────────────────────
SWAP_AXES = False        # True の場合、縦軸=pot1 / 横軸=pot2 を、縦軸=pot2 / 横軸=pot1 に入れ替え
INVERT_X = True         # 横軸の正負/左右反転
INVERT_Y = True         # 縦軸の正負/上下反転

POT_X_NEUTRAL = 540     # 横軸（ラダー）の中立点 (ADC 12bit値)
POT_Y_NEUTRAL = 1805    # 縦軸（エレベーター）の中立点 (ADC 12bit値)

POT_X_MAX_DEV = 2048     # 横軸の最大偏移幅（中立点からの最大ADCズレ量、スケール調整に用いる）
POT_Y_MAX_DEV = 2048     # 縦軸の最大偏移幅（中立点からの最大ADCズレ量、スケール調整に用いる）

STEER_X_MAX_DEG = 30.0   # 横軸最大表示舵角（度）
STEER_Y_MAX_DEG = 30.0   # 縦軸最大表示舵角（度）

MAX_TRAIL_DOTS = 15      # 軌跡（残像）として描画する点の最大数
# ─────────────────────────────────────────────────────────────────────────────


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_queue: queue.Queue[str]) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            with serial.Serial(self.port, self.baud, timeout=0.2, dsrdtr=False, rtscts=False) as ser:
                ser.setDTR(False)
                ser.setRTS(False)
                while not self._stop_event.is_set():
                    line = ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        self.out_queue.put(line)
        except Exception as exc:
            self.out_queue.put(f"[error] {exc}")


class SpeedPanel:
    def __init__(self, parent: tk.Widget, title: str, color: str, scale_max: float) -> None:
        self.scale_max = scale_max
        self.color = color
        self.last_speed = 0.0
        self.last_update_at = 0.0
        self.canvas_width = 240

        frame = tk.Frame(parent, bg="#ffffff", bd=0, highlightthickness=1, highlightbackground="#d8e0e5")
        frame.pack(fill=tk.X, pady=2)

        # 1行にタイトルと数値を綺麗に並べる
        header_frame = tk.Frame(frame, bg="#ffffff")
        header_frame.pack(fill=tk.X, padx=6, pady=(3, 1))

        tk.Label(
            header_frame,
            text=title.split(" (")[1].replace(")", "") if " (" in title else title, # シンプルにAirspeedやWindspeedのみに
            font=("Yu Gothic UI", 9, "bold"),
            bg="#ffffff",
            fg="#17232d",
        ).pack(side=tk.LEFT)

        self.value_var = tk.StringVar(value="0.0 m/s")
        self.state_var = tk.StringVar(value="STALE")

        tk.Label(
            header_frame,
            textvariable=self.value_var,
            font=("Consolas", 10, "bold"),
            bg="#ffffff",
            fg="#111111",
        ).pack(side=tk.RIGHT)

        # バー表示部を省スペース化
        canvas = tk.Canvas(frame, width=self.canvas_width, height=14, bg="#eef3f6", highlightthickness=0)
        canvas.pack(padx=6, pady=(0, 4))
        bg_rect = canvas.create_rectangle(0, 2, self.canvas_width, 12, fill="#dde6ec", outline="")
        self.bar = canvas.create_rectangle(0, 2, 0, 12, fill=color, outline="")
        canvas.create_text(self.canvas_width - 4, 7, text=f"{scale_max:.0f}", anchor="e", fill="#5a6976", font=("Consolas", 7))
        _ = bg_rect

        self.canvas = canvas

    def update(self, speed: float) -> None:
        self.last_speed = speed
        self.last_update_at = time.monotonic()

    def refresh(self) -> None:
        state = "OK" if (time.monotonic() - self.last_update_at) <= STALE_TIMEOUT_SEC else "STALE"
        speed = self.last_speed
        clamped = max(0.0, min(self.scale_max, speed))
        width = self.canvas_width * (clamped / self.scale_max) if self.scale_max > 0 else 0.0
        self.canvas.coords(self.bar, 0, 2, width, 12)
        self.canvas.itemconfigure(self.bar, fill=self.color if state == "OK" else "#a8b3bc")
        self.value_var.set(f"{speed:.1f} m/s ({state})")


class SteeringPanel:
    def __init__(self, parent: tk.Widget) -> None:
        self.points_history: list[tuple[float, float]] = []  # (steer_x_deg, steer_y_deg) の履歴

        frame = tk.Frame(parent, bg="#ffffff", bd=0, highlightthickness=1, highlightbackground="#d8e0e5")
        frame.pack(fill=tk.BOTH, expand=True, pady=2)

        # 操舵角用二次元 Canvas (220x220) に綺麗に拡大、無駄なテキストを完全排除
        self.canvas_size = 220
        self.canvas = tk.Canvas(frame, width=self.canvas_size, height=self.canvas_size, bg="#f8fafc", highlightthickness=0)
        self.canvas.pack(padx=6, pady=6, expand=True)

        # ガイド線の描画
        mid = self.canvas_size / 2
        self.canvas.create_line(0, mid, self.canvas_size, mid, fill="#cbd5e1", dash=(2, 2))  # 横軸
        self.canvas.create_line(mid, 0, mid, self.canvas_size, fill="#cbd5e1", dash=(2, 2))  # 縦軸

        # 現在位置を示すマーカー（赤丸）
        self.current_marker = self.canvas.create_oval(0, 0, 0, 0, fill="#ef4444", outline="#ffffff", width=1.5)

    def update_steer(self, pot1: int, pot2: int) -> None:
        # 1. 軸の入れ替え設定の適用
        raw_x = pot2 if SWAP_AXES else pot1  # 横軸（Rudder）
        raw_y = pot1 if SWAP_AXES else pot2  # 縦軸（Elevator）

        # 2. 中立点からのズレを算出
        dev_x = raw_x - POT_X_NEUTRAL
        dev_y = raw_y - POT_Y_NEUTRAL

        # 3. 反転処理
        if INVERT_X:
            dev_x = -dev_x
        if INVERT_Y:
            dev_y = -dev_y

        # 4. 舵角 (度数) へのスケール変換
        steer_x_deg = (dev_x / POT_X_MAX_DEV) * STEER_X_MAX_DEG
        steer_y_deg = (dev_y / POT_Y_MAX_DEV) * STEER_Y_MAX_DEG

        # クランプ処理
        steer_x_deg = max(-STEER_X_MAX_DEG, min(STEER_X_MAX_DEG, steer_x_deg))
        steer_y_deg = max(-STEER_Y_MAX_DEG, min(STEER_Y_MAX_DEG, steer_y_deg))

        # 履歴記録
        self.points_history.append((steer_x_deg, steer_y_deg))
        if len(self.points_history) > MAX_TRAIL_DOTS:
            self.points_history.pop(0)

        return steer_x_deg, steer_y_deg

    def refresh(self) -> None:
        # 古い描画痕跡（残像タグのもののみ）を消去
        self.canvas.delete("trail")

        mid = self.canvas_size / 2
        half_range_x = STEER_X_MAX_DEG
        half_range_y = STEER_Y_MAX_DEG

        # 残像（軌跡）の描画
        num_points = len(self.points_history)
        for i, (sx, sy) in enumerate(self.points_history[:-1]):
            # キャンバス座標系への変換 (sx は右方向がプラス, sy は上方向がプラス)
            cx = mid + (sx / half_range_x) * mid
            cy = mid - (sy / half_range_y) * mid  # Tkinterは下方向がY軸プラスのためマイナス

            # 古い点ほど小さく、薄く（透明度またはカラー調整で水色〜青にグラデーション）
            ratio = (i + 1) / num_points
            r = max(1.5, ratio * 3.5)
            
            # グラデーションカラー (薄い青から濃い青へ)
            blue_val = int(140 + ratio * 115)  # 140 -> 255
            color_hex = f"#93c5fd" if ratio < 0.4 else (f"#60a5fa" if ratio < 0.7 else f"#2563eb")

            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=color_hex, outline="", tags="trail")

        # 最新位置の描画
        if self.points_history:
            sx, sy = self.points_history[-1]
            cx = mid + (sx / half_range_x) * mid
            cy = mid - (sy / half_range_y) * mid
            r = 6.0
            self.canvas.coords(self.current_marker, cx - r, cy - r, cx + r, cy + r)
            self.canvas.tag_raise(self.current_marker)


class App:
    def __init__(self, root: tk.Tk, port: str, baud: int, max_speed: float) -> None:
        self.root = root
        self.root.title("PONS Flight Monitor")
        self.root.geometry("280x420")  # 高さを380から420に少し広げて、速度計3つが綺麗に収まるようにします
        self.root.configure(bg="#f3f6f8")

        self.queue: queue.Queue[str] = queue.Queue()
        self.reader = SerialReader(port, baud, self.queue)

        # 縦並びのメインレイアウト（pack方向はTOP、fillはX）
        main_layout = tk.Frame(root, bg="#f3f6f8")
        main_layout.pack(fill=tk.BOTH, expand=True, padx=6, pady=(6, 0))

        # 1. 速度計3つ（上部） - 風速は10.0m/sを最大に固定、対気速度はmax_speed
        self.air_panel = SpeedPanel(main_layout, "main beam (Airspeed)", "#2d7ff9", max_speed)
        self.wind1_panel = SpeedPanel(main_layout, "wing L (Windspeed 1)", "#ef7d32", 10.0)
        self.wind2_panel = SpeedPanel(main_layout, "wing R (Windspeed 2)", "#10b981", 10.0)  # きれいなエメラルドグリーン

        # 2. 2次元操舵パネル（下部）
        self.steer_panel = SteeringPanel(main_layout)

        # 最新データ保持用のバッファ
        self.last_air = 0.0
        self.last_wind = 0.0
        self.last_wind1 = 0.0
        self.last_wind2 = 0.0
        self.last_pot1 = 0
        self.last_pot2 = 0
        self.last_steer_x = 0.0
        self.last_steer_y = 0.0

        # CSV 保存用の初期化
        import csv
        from datetime import datetime
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"telemetry_{timestamp_str}.csv"
        try:
            self.csv_file = open(self.csv_filename, mode="w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "timestamp", "airspeed_mps", "wind_combined_mps", 
                "wind1_mps", "wind2_mps", "pot1", "pot2", 
                "steer_x_deg", "steer_y_deg"
            ])
            self.csv_file.flush()
            print(f"[CSV Logger] Logging to {self.csv_filename}")
        except Exception as e:
            print(f"[CSV Logger] Failed to open CSV file: {e}")
            self.csv_file = None

        self.status_var = tk.StringVar(value=f"port={port} baud={baud}")
        self.line_var = tk.StringVar(value="waiting for serial data...")

        footer = tk.Frame(root, bg="#f3f6f8")
        footer.pack(fill=tk.X, padx=6, pady=(0, 4), side=tk.BOTTOM)

        tk.Label(
            footer,
            textvariable=self.status_var,
            font=("Consolas", 8),
            bg="#f3f6f8",
            fg="#2f3d49",
            anchor="w",
        ).pack(fill=tk.X, pady=(1, 1))

        tk.Label(
            footer,
            textvariable=self.line_var,
            font=("Consolas", 7),
            bg="#dde6ec",
            fg="#24313c",
            anchor="w",
            padx=4,
            pady=2,
        ).pack(fill=tk.X)

        self.reader.start()
        self.root.after(50, self.poll_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def poll_queue(self) -> None:
        while True:
            try:
                line = self.queue.get_nowait()
            except queue.Empty:
                break

            self.line_var.set(line)
            if line.startswith("[error]"):
                self.status_var.set(line)
                continue

            air_match = AIR_PATTERN.search(line)
            wind_match = WIND_PATTERN.search(line)
            wind1_match = WIND1_PATTERN.search(line)
            wind2_match = WIND2_PATTERN.search(line)
            disp_match = DISP_PATTERN.search(line)
            
            matched = False
            if air_match:
                self.last_air = float(air_match.group(1))
                self.air_panel.update(self.last_air)
                matched = True
            
            # w1 / w2 の個別データがあればそれぞれ展開。なければ従来の統合wind値（ロガー等）をwind1にフォールバック
            if wind1_match:
                self.last_wind1 = float(wind1_match.group(1))
                self.wind1_panel.update(self.last_wind1)
                matched = True
            elif wind_match:
                self.last_wind = float(wind_match.group(1))
                self.last_wind1 = self.last_wind
                self.wind1_panel.update(self.last_wind1)
                matched = True

            if wind2_match:
                self.last_wind2 = float(wind2_match.group(1))
                self.wind2_panel.update(self.last_wind2)
                matched = True
            
            if disp_match:
                self.last_pot1 = int(disp_match.group(1))
                self.last_pot2 = int(disp_match.group(2))
                self.last_steer_x, self.last_steer_y = self.steer_panel.update_steer(self.last_pot1, self.last_pot2)
                matched = True
                
            if air_match or wind_match or wind1_match or wind2_match or disp_match:
                self.status_var.set("receiving")

            if matched and self.csv_file:
                from datetime import datetime
                now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                try:
                    self.csv_writer.writerow([
                        now_str,
                        self.last_air,
                        self.last_wind,
                        self.last_wind1,
                        self.last_wind2,
                        self.last_pot1,
                        self.last_pot2,
                        f"{self.last_steer_x:.2f}",
                        f"{self.last_steer_y:.2f}"
                    ])
                    self.csv_file.flush()
                except Exception as e:
                    print(f"[CSV Logger] Write error: {e}")

        self.air_panel.refresh()
        self.wind1_panel.refresh()
        self.wind2_panel.refresh()
        self.steer_panel.refresh()
        self.root.after(50, self.poll_queue)

    def close(self) -> None:
        self.reader.stop()
        if hasattr(self, "csv_file") and self.csv_file:
            try:
                self.csv_file.close()
                print(f"[CSV Logger] Closed {self.csv_filename} successfully.")
            except Exception:
                pass
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="ESP-NOW dual monitor visualizer")
    parser.add_argument("--port", default="COM16", help="Serial port name")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--max-speed", type=float, default=15.0, help="Bar graph full-scale speed [m/s]")
    args = parser.parse_args()

    root = tk.Tk()
    App(root, args.port, args.baud, args.max_speed)
    root.mainloop()


if __name__ == "__main__":
    main()
