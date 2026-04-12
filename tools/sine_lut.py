import math

DEPTH = 256      # 2^8 深度
WIDTH = 8        # 8位宽
MAX_VAL = 255

with open("sine_wave_256x8.hex", "w") as f:
    for i in range(DEPTH):
        # 计算 0-2pi
        val = (math.sin(2 * math.pi * i / DEPTH) + 1) / 2
        int_val = int(val * MAX_VAL)
        # 以 2 位 16 进制格式写入 (例如: FF)
        f.write(f"{int_val:02X}\n")

print("Generated sine_wave_256x8.hex successfully.")