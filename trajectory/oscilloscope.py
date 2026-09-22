import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ==============================================================================
# CẤU HÌNH UART
# ==============================================================================
UART_PORT = "COM4"
UART_BAUD = 921600

# Khởi tạo bộ đệm lưu trữ 200 điểm dữ liệu gần nhất để tạo hiệu ứng cuộn
MAX_POINTS = 200
y_data = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)

# ==============================================================================
# KHỞI TẠO ĐỒ THỊ
# ==============================================================================
fig, ax = plt.subplots(figsize=(10, 5))
line, = ax.plot(y_data, '-g', lw=1.5)  # Đường biểu diễn màu xanh lá

ax.set_ylim(0, 5.0)  # Giới hạn trục Y (thay đổi tùy theo dải tín hiệu của bạn)
ax.set_title("UART Real-time Oscilloscope")
ax.set_xlabel("Số mẫu")
ax.set_ylabel("Giá trị")
ax.grid(True, linestyle='--', alpha=0.6)

# Mở kết nối UART
try:
    ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.1)
    print(f"Đã mở cổng {UART_PORT}. Đang lắng nghe dữ liệu...")
except Exception as e:
    print(f"Không thể mở cổng UART: {e}")
    exit()

# ==============================================================================
# HÀM CẬP NHẬT ĐỒ THỊ LIÊN TỤC
# ==============================================================================
def update_plot(frame):
    # Đọc tất cả các dòng hiện có trong bộ đệm UART
    while ser.in_waiting > 0:
        try:
            line_str = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Nếu có dữ liệu, thử ép kiểu sang số thực (float) để vẽ
            if line_str:
                value = float(line_str)
                y_data.append(value)
                
        except ValueError:
            # Bỏ qua các chuỗi văn bản (như "END", chữ cái, rác)
            pass
            
    # Cập nhật đường vẽ với dữ liệu mới
    line.set_ydata(y_data)
    return line,

# Chạy vòng lặp cập nhật đồ thị mỗi 20ms
ani = animation.FuncAnimation(fig, update_plot, interval=20, blit=True, cache_frame_data=False)

try:
    # Nếu firmware của bạn vẫn cần lệnh START để bắt đầu gửi dữ liệu, bỏ comment dòng dưới:
    # ser.write(b"START") 
    
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Đã đóng kết nối UART.")