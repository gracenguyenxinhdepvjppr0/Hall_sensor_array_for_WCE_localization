import serial
import time
import numpy as np

# Cấu hình UART
UART_PORT = "COM4" 
UART_BAUD = 921600 
UART_TIMEOUT = 10000000
  

# Thông số mảng cảm biến
SENSOR_ROWS = 64  # Số lượng cảm biến trong 1 lần quét (8 hàng x 8 cột)
NUM_REPEATS = 5   # Vi điều khiển lặp quét 5 lần

def read_and_average_data():
    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=UART_TIMEOUT)
        time.sleep(1)
        
        ser.reset_input_buffer()
        print("Đang gửi lệnh: START")
        ser.write(b"START")
        
        # Tạo mảng 2 chiều để chứa dữ liệu: 5 cột (5 lần lặp), mỗi cột 64 hàng
        data_matrix = [[] for _ in range(NUM_REPEATS)]
        current_repeat = 0
        
        print("Đang đọc và tính toán dữ liệu...")
        while True:
            raw_data = ser.readline()
            
            if not raw_data:
                print("Lỗi: Timeout - Không nhận được chuỗi END.")
                break
                
            line = raw_data.decode("utf-8", errors="ignore").strip()
            
            if line == "END":
                break
            
            try:
                voltage = float(line)
                
                # Phân bổ dữ liệu vào từng lần lặp tương ứng
                if current_repeat < NUM_REPEATS:
                    data_matrix[current_repeat].append(voltage)
                    
                    # Nếu đã đọc đủ 64 cảm biến cho vòng lặp hiện tại, chuyển sang vòng tiếp theo[cite: 1]
                    if len(data_matrix[current_repeat]) == SENSOR_ROWS:
                        current_repeat += 1
                        
            except ValueError:
                pass # Bỏ qua các chuỗi không phải là số
        
        ser.close()
        
        # ==========================================
        # TÍNH TOÁN TRUNG BÌNH VÀ IN KẾT QUẢ
        # ==========================================
        # Kiểm tra xem có nhận đủ dữ liệu không (5 lần x 64 = 320 mẫu)
        is_valid = True
        for i, col in enumerate(data_matrix):
            if len(col) != SENSOR_ROWS:
                print(f"[Cảnh báo] Lần quét thứ {i+1} chỉ nhận được {len(col)}/{SENSOR_ROWS} mẫu.")
                is_valid = False
                
        if is_valid:
            # Chuyển đổi sang mảng numpy để tính toán[cite: 1]
            np_data = np.array(data_matrix)
            
            # Tính trung bình cộng theo cột (axis=0 tính trung bình của 5 lần quét cho từng cảm biến)[cite: 1]
            averaged_data = np.mean(np_data, axis=0)
            
            print("\n" + "="*50)
            print(f"ĐÃ NHẬN ĐỦ VÀ TÍNH TRUNG BÌNH CHO {SENSOR_ROWS} CẢM BIẾN")
            print("="*50)
            
            # In kết quả đã làm tròn 4 chữ số thập phân
            for i, val in enumerate(averaged_data):
                print(f"Cảm biến {i+1:02d}: {val:.4f} V")
                
            print("="*50)
            
    except Exception as e:
        print(f"Lỗi UART: {e}")

if __name__ == "__main__":
    read_and_average_data()