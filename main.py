"""
============================================================
SCRIPT PHIA MAY TINH - HE THONG BO DAM 2 CHIEU VOI ESP32
(Khong dung numpy - dung sounddevice RawStream lam viec truc tiep voi bytes)
============================================================
Cai dat thu vien can thiet (chi can chay 1 lan):
  pip install sounddevice

Chay:
  python computer_server.py
============================================================
"""

import socket
import threading
import sounddevice as sd

# ================== CAU HINH ==================
UDP_PORT = 3333            # phai trung voi UDP_PORT trong code ESP32
SAMPLE_RATE = 16000        # phai trung voi SAMPLE_RATE trong code ESP32
CHANNELS = 1
SAMPLES_PER_PACKET = 512   # so sample moi goi UDP gui di (giong ESP32)

# Bien luu dia chi IP cua ESP32, tu dong nhan duoc tu goi tin dau tien no gui den
esp32_address = None
esp32_lock = threading.Lock()

# Socket UDP dung chung cho ca gui va nhan
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))

print(f"[INFO] Dang lang nghe UDP tren cong {UDP_PORT} ...")


# ============================================================
# LUONG 1: NHAN AM THANH TU ESP32 -> PHAT RA LOA MAY TINH
# ============================================================
def receive_from_esp32_and_play():
    global esp32_address

    # RawOutputStream ghi truc tiep bytes, khong can chuyen qua numpy array
    with sd.RawOutputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, dtype="int16") as out_stream:
        while True:
            data, addr = sock.recvfrom(4096)

            # Ghi nho dia chi ESP32 de biet gui am thanh nguoc lai dau
            with esp32_lock:
                esp32_address = addr

            out_stream.write(data)


# ============================================================
# LUONG 2: THU AM TU MIC MAY TINH -> GUI VE ESP32 QUA UDP
# ============================================================
def record_from_mic_and_send():
    def callback(indata, frames, time_info, status):
        global esp32_address
        if status:
            print("[WARN]", status)

        with esp32_lock:
            target = esp32_address

        if target is not None:
            # indata la buffer bytes tho (cffi buffer), chuyen sang bytes de gui qua socket
            sock.sendto(bytes(indata), target)

    with sd.RawInputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype="int16",
        blocksize=SAMPLES_PER_PACKET,
        callback=callback,
    ):
        print("[INFO] Dang thu am tu mic may tinh, san sang gui ve ESP32...")
        while True:
            sd.sleep(1000)


# ============================================================
# CHAY 2 LUONG SONG SONG
# ============================================================
if __name__ == "__main__":
    t1 = threading.Thread(target=receive_from_esp32_and_play, daemon=True)
    t2 = threading.Thread(target=record_from_mic_and_send, daemon=True)

    t1.start()
    t2.start()

    print("[INFO] He thong dang chay. Nhan Ctrl+C de dung.")
    try:
        while True:
            threading.Event().wait(1)
    except KeyboardInterrupt:
        print("\n[INFO] Da dung chuong trinh.")