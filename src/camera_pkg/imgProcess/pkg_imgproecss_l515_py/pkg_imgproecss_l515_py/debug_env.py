# (debug_env.py 文件顶部原有内容)
import sys
import os

print("--- Python Environment Debug ---")
print(f"Python Executable: {sys.executable}")
print("sys.path:")
for p in sys.path:
    print(f"  {p}")

try:
    import mediapipe
    print("\n'mediapipe' module found!")
    print(f"  Location: {mediapipe.__file__}")
except ImportError:
    print("\n'mediapipe' module NOT found!")

print("--- End Debug ---")

# --- 添加这部分代码 ---
def main(args=None):
    # 这里的逻辑已经在文件顶部执行，main函数仅作为入口点
    pass

if __name__ == '__main__':
    main()