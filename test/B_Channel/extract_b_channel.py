import cv2
import os
import sys
from pathlib import Path

def extract_b_channel(video_path):
    """
    将视频的B通道提取出来并保存为新的视频文件
    
    Args:
        video_path: 输入视频文件路径
    """
    # 检查文件是否存在
    if not os.path.exists(video_path):
        print(f"错误：文件 '{video_path}' 不存在！")
        return False
    
    # 获取文件路径信息
    video_path = Path(video_path)
    video_dir = video_path.parent
    video_filename = video_path.stem
    video_extension = video_path.suffix
    
    # 构建输出文件名
    output_filename = f"B_{video_filename}.mp4"
    output_path = video_dir / output_filename
    
    # 打开视频文件
    cap = cv2.VideoCapture(str(video_path))
    
    if not cap.isOpened():
        print(f"错误：无法打开视频文件 '{video_path}'！")
        return False
    
    # 获取视频信息
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"视频信息:")
    print(f"  文件名: {video_filename}{video_extension}")
    print(f"  帧率: {fps} FPS")
    print(f"  分辨率: {width}x{height}")
    print(f"  总帧数: {total_frames}")
    print(f"  输出文件: {output_filename}")
    
    # 创建视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height), isColor=True)
    
    if not out.isOpened():
        print("错误：无法创建输出视频文件！")
        cap.release()
        return False
    
    frame_count = 0
    
    print("开始处理视频...")
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        
        # 提取B通道
        # OpenCV读取的通道顺序是BGR，所以B通道是第一个通道
        b_channel = frame[:, :, 0]  # B通道（索引0）
        
        # 将单通道图像转换为三通道（BGR）图像
        # B通道在三个通道中都显示蓝色，保持亮度信息
        b_frame = cv2.merge([b_channel, np.zeros_like(b_channel), np.zeros_like(b_channel)])
        
        # 写入输出视频
        out.write(b_frame)
        
        frame_count += 1
        
        # 显示处理进度
        if frame_count % 100 == 0:
            progress = (frame_count / total_frames) * 100
            print(f"处理进度: {frame_count}/{total_frames} 帧 ({progress:.1f}%)")
    
    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
    print(f"\n处理完成！")
    print(f"已保存为: {output_path}")
    print(f"处理总帧数: {frame_count}")
    
    return True

def extract_b_channel_grayscale(video_path):
    """
    将视频的B通道提取为灰度视频（可选）
    """
    # 检查文件是否存在
    if not os.path.exists(video_path):
        print(f"错误：文件 '{video_path}' 不存在！")
        return False
    
    # 获取文件路径信息
    video_path = Path(video_path)
    video_dir = video_path.parent
    video_filename = video_path.stem
    
    # 构建输出文件名（灰度版本）
    output_filename = f"B_{video_filename}_grayscale.mp4"
    output_path = video_dir / output_filename
    
    # 打开视频文件
    cap = cv2.VideoCapture(str(video_path))
    
    if not cap.isOpened():
        print(f"错误：无法打开视频文件 '{video_path}'！")
        return False
    
    # 获取视频信息
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"创建灰度版本...")
    print(f"  输出文件: {output_filename}")
    
    # 创建视频写入器（单通道）
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height), isColor=False)
    
    if not out.isOpened():
        print("错误：无法创建输出视频文件！")
        cap.release()
        return False
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        
        # 提取B通道
        b_channel = frame[:, :, 0]
        
        # 直接写入单通道图像
        out.write(b_channel)
        
        frame_count += 1
    
    # 释放资源
    cap.release()
    out.release()
    
    print(f"灰度版本已保存为: {output_path}")
    
    return True

# 如果直接运行脚本，处理指定视频
if __name__ == "__main__":
    import numpy as np
    
    if len(sys.argv) > 1:
        # 使用命令行参数指定的视频文件
        video_path = sys.argv[1]
    else:
        # 如果没有提供参数，使用当前目录下的示例视频
        # 或者让用户输入
        video_path = input("请输入视频文件路径: ").strip()
        if not video_path:
            print("使用默认示例（请确保有视频文件）")
            # 可以修改为默认路径
            video_path = "input_video.mp4"
    
    try:
        # 主要处理函数（蓝色版本）
        success = extract_b_channel(video_path)
        
        if success:
            # 询问是否创建灰度版本
            create_gray = input("\n是否同时创建灰度版本？(y/n): ").lower()
            if create_gray == 'y':
                extract_b_channel_grayscale(video_path)
    except Exception as e:
        print(f"处理过程中出现错误: {e}")
        import traceback
        traceback.print_exc()