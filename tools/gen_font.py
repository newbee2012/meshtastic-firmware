# -*- coding: utf-8 -*-
import struct
import sys
try:
    from PIL import Image, ImageFont, ImageDraw
except ImportError:
    print("错误：请先安装 pillow 库 (pip install pillow)")
    sys.exit(1)

# ================= 配置区域 (保持您当前的设置) =================
FONT_FILE = "simsun.ttc"
FONT_SIZE = 13           
OUTPUT_FILE = "../src/graphics/fonts/ChineseFont.h"
Y_OFFSET = 1              # 垂直对齐偏移量
# ===============================================================

# GB2312 编码常量
GB_START_HIGH = 0xA1  # 区码起始字节 (1区)
GB_END_HIGH = 0xF7    # 区码结束字节 (87区)
GB_START_LOW = 0xA1   # 位码起始字节 (1位)
GB_END_LOW = 0xFE     # 位码结束字节 (94位)
GB_POSITIONS_PER_ZONE = (GB_END_LOW - GB_START_LOW + 1) # 94

def get_bitmap(char, font, debug=False):
    """生成 16x16 点阵数据 (32字节) - 自动居中/基线对齐"""
    img = Image.new("1", (16, 16), 0)
    draw = ImageDraw.Draw(img)
    
    bbox = draw.textbbox((0, 0), char, font=font)
    text_width = bbox[2] - bbox[0]
    x = (16 - text_width) // 2 - bbox[0]
    y = Y_OFFSET 
    
    draw.text((x, y), char, font=font, fill=1)
    
    bitmap = []
    for r in range(16):
        byte1 = 0
        byte2 = 0
        for c in range(8):
            if img.getpixel((c, r)):
                byte1 |= (1 << c)
        for c in range(8, 16):
            if img.getpixel((c, r)):
                byte2 |= (1 << (c - 8))
        bitmap.append(byte1)
        bitmap.append(byte2)
    return bitmap

def main():
    try:
        font = ImageFont.truetype(FONT_FILE, FONT_SIZE)
    except IOError:
        print("错误：找不到字体文件 {}。".format(FONT_FILE))
        sys.exit(1)
        
    chars_data = []
    map_table = [] 
    print(f"正在生成全范围 GB2312 字库 (Zone 1-87, {hex(GB_START_HIGH)} 到 {hex(GB_END_HIGH)})...")
    
    count = 0
    
    # 遍历所有区码字节 (0xA1 到 0xF7)
    for high_byte in range(GB_START_HIGH, GB_END_HIGH + 1):
        # 遍历所有位码字节 (0xA1 到 0xFE)
        for low_byte in range(GB_START_LOW, GB_END_LOW + 1):
            
            gb_code = bytes([high_byte, low_byte])
            
            try:
                char = gb_code.decode("gb2312")
                bitmap = get_bitmap(char, font, debug=False)
                
                uni_code = ord(char)
                gb_int = high_byte * 256 + low_byte
                
                chars_data.append(bitmap)
                map_table.append((uni_code, gb_int))
                count += 1
                
            except (UnicodeDecodeError, Exception):
                # 解码失败（未分配的码位或字体无法渲染），填充空数据，保证数组连续性
                chars_data.append([0]*32)
                pass

    print("实际生成了 {} 个点阵 (应为 {} 个)，共 {} 个有效字符/汉字。".format(
        len(chars_data), 
        (GB_END_HIGH - GB_START_HIGH + 1) * GB_POSITIONS_PER_ZONE,
        count)
    )

    map_table.sort(key=lambda x: x[0])

    # === 写入 .h 文件 ===
    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write("#ifndef CHINESEFONT_H\n#define CHINESEFONT_H\n\n")
        f.write("#include <Arduino.h>\n\n")
        
        f.write("// GB2312 汉字点阵 (Zone 1-87, A1A1-F7FE)\n")
        f.write("const uint8_t GB2312_FontData[{} * 32] PROGMEM = {{\n".format(len(chars_data)))
        for idx, bitmap in enumerate(chars_data):
            hex_str = ", ".join(["0x{:02X}".format(b) for b in bitmap])
            f.write("    {}, // idx {}\n".format(hex_str, idx))
        f.write("};\n\n")
        
        f.write("struct CodeMap {\n    uint16_t unicode;\n    uint16_t gbCode;\n};\n\n")
        f.write("const CodeMap UTF8ToGB_Table[{}] PROGMEM = {{\n".format(len(map_table)))
        for uni, gb in map_table:
            f.write("    {{0x{:04X}, 0x{:04X}}},\n".format(uni, gb))
        f.write("};\n\n")
        
        # 写入宏定义供 C++ 使用
        f.write("#define GB_START_HIGH 0xA1\n")
        f.write("#define GB_START_LOW 0xA1\n")
        f.write("#define GB_POSITIONS_PER_ZONE 94\n")
        f.write("#define GB_END_HIGH 0xF7\n")
        f.write("#define GB_END_LOW 0xFE\n")
        
        f.write("#endif\n")

if __name__ == "__main__":
    main()