#!/usr/bin/env python3
"""
YOLOv8训练脚本
输入尺寸：160x160
输出：ONNX格式模型
针对边缘设备优化（CPU X1 1GHz，0.37GB RAM）
"""

import os
import yaml
import shutil
from pathlib import Path
from ultralytics import YOLO
import cv2
import random

def create_dataset_structure():
    """创建YOLO格式的数据集结构"""
    dataset_dir = Path("dataset")
    
    # 创建目录结构
    directories = [
        dataset_dir / "images" / "train",
        dataset_dir / "images" / "val",
        dataset_dir / "labels" / "train",
        dataset_dir / "labels" / "val",
    ]
    
    for dir_path in directories:
        dir_path.mkdir(parents=True, exist_ok=True)
    
    return dataset_dir

def get_class_mapping():
    """获取类别映射"""
    # 基于train_data目录的类别
    class_dirs = [
        "A-firearms", "B-explosive", "C-dagger", "D-spontoon", "E-fire_axe",
        "F-first_aid_kit", "G-flashlight", "H-intercom", "I-bulletproof_vest",
        "J-telescope", "K-helmet", "L-fire_engine", "M-ambulance", 
        "N-armoured", "O-motorcycle"
    ]
    
    class_mapping = {}
    for idx, class_name in enumerate(class_dirs):
        # 简化类别名称
        simple_name = class_name.split('-')[-1]
        class_mapping[class_name] = {
            'id': idx,
            'name': simple_name
        }
    
    return class_mapping

def generate_yolo_labels(image_path, class_id, output_label_path):
    """
    生成YOLO格式的标注文件
    假设物体占据整个图像中心区域
    """
    # 读取图像获取尺寸
    img = cv2.imread(str(image_path))
    if img is None:
        return False
    
    height, width = img.shape[:2]
    
    # 假设物体占据图像中心80%的区域
    x_center = 0.5
    y_center = 0.5
    bbox_width = 0.8
    bbox_height = 0.8
    
    # 写入YOLO格式标注
    with open(output_label_path, 'w') as f:
        f.write(f"{class_id} {x_center} {y_center} {bbox_width} {bbox_height}")
    
    return True

def prepare_dataset():
    """准备数据集"""
    dataset_dir = create_dataset_structure()
    class_mapping = get_class_mapping()
    
    train_data_dir = Path("train_data")
    all_images = []
    
    # 收集所有图像
    for class_dir_name, class_info in class_mapping.items():
        class_dir = train_data_dir / class_dir_name
        if not class_dir.exists():
            print(f"警告: 目录不存在: {class_dir}")
            continue
        
        # 获取所有jpg文件
        image_files = list(class_dir.glob("*.jpg"))
        print(f"类别 {class_info['name']}: 找到 {len(image_files)} 张图片")
        
        for img_path in image_files:
            all_images.append((img_path, class_info['id'], class_info['name']))
    
    print(f"总共找到 {len(all_images)} 张图片")
    
    # 随机打乱并分割训练/验证集
    random.shuffle(all_images)
    split_idx = int(len(all_images) * 0.8)  # 80%训练，20%验证
    
    train_images = all_images[:split_idx]
    val_images = all_images[split_idx:]
    
    # 处理训练集
    train_count = 0
    for img_path, class_id, class_name in train_images:
        # 复制图像
        dest_img_path = dataset_dir / "images" / "train" / img_path.name
        shutil.copy2(img_path, dest_img_path)
        
        # 生成标注
        label_name = img_path.stem + ".txt"
        label_path = dataset_dir / "labels" / "train" / label_name
        if generate_yolo_labels(img_path, class_id, label_path):
            train_count += 1
    
    # 处理验证集
    val_count = 0
    for img_path, class_id, class_name in val_images:
        # 复制图像
        dest_img_path = dataset_dir / "images" / "val" / img_path.name
        shutil.copy2(img_path, dest_img_path)
        
        # 生成标注
        label_name = img_path.stem + ".txt"
        label_path = dataset_dir / "labels" / "val" / label_name
        if generate_yolo_labels(img_path, class_id, label_path):
            val_count += 1
    
    print(f"数据集准备完成: {train_count} 训练图片, {val_count} 验证图片")
    
    return dataset_dir, class_mapping

def create_data_yaml(dataset_dir, class_mapping):
    """创建data.yaml配置文件"""
    data_yaml = {
        'path': str(dataset_dir.absolute()),
        'train': 'images/train',
        'val': 'images/val',
        'names': {idx: info['name'] for idx, info in enumerate(class_mapping.values())}
    }
    
    yaml_path = dataset_dir / "data.yaml"
    with open(yaml_path, 'w') as f:
        yaml.dump(data_yaml, f, default_flow_style=False)
    
    print(f"创建配置文件: {yaml_path}")
    return yaml_path

def check_gpu_availability():
    """检查GPU可用性"""
    try:
        import torch
        cuda_available = torch.cuda.is_available()
        if cuda_available:
            gpu_count = torch.cuda.device_count()
            gpu_name = torch.cuda.get_device_name(0) if gpu_count > 0 else "未知"
            print(f"✅ GPU可用: {gpu_count}个设备")
            print(f"   GPU名称: {gpu_name}")
            print(f"   CUDA版本: {torch.version.cuda}")
            return True, gpu_count
        else:
            print("⚠️  GPU不可用，将使用CPU训练")
            print("   注意: CPU训练速度较慢，建议安装GPU版本的PyTorch")
            return False, 0
    except Exception as e:
        print(f"❌ 检查GPU时出错: {e}")
        return False, 0

def train_yolo_model(data_yaml_path, use_gpu=True):
    """训练YOLO模型"""
    print("开始训练YOLOv8模型...")
    
    # 检查GPU可用性
    gpu_available, gpu_count = check_gpu_availability()
    
    # 确定训练设备
    if use_gpu and gpu_available and gpu_count > 0:
        device = 'cuda'  # 使用第一个GPU
        print(f"使用GPU进行训练 (设备: cuda:0)")
        batch_size = 16  # GPU可以处理更大的批量
        workers = 4      # GPU可以处理更多worker
    else:
        device = 'cpu'
        print("使用CPU进行训练")
        batch_size = 8   # CPU批量较小
        workers = 2      # CPU worker较少
    
    # 使用YOLOv8n（最轻量级）
    model = YOLO('yolov8n.yaml')  # 从配置文件创建新模型
    
    # 训练参数（针对边缘设备优化）
    results = model.train(
        data=str(data_yaml_path),
        epochs=50,  # 训练周期
        imgsz=160,  # 输入尺寸160x160
        batch=batch_size,    # 批量大小（根据设备调整）
        workers=workers,  # worker数量（根据设备调整）
        device=device,  # 使用GPU或CPU
        patience=10,   # 早停耐心值
        lr0=0.01,      # 初始学习率
        lrf=0.01,      # 最终学习率因子
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3,
        warmup_momentum=0.8,
        box=7.5,
        cls=0.5,
        dfl=1.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=0.0,
        translate=0.1,
        scale=0.5,
        shear=0.0,
        perspective=0.0,
        flipud=0.0,
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.0 if device == 'cpu' else 0.5,  # CPU禁用mixup，GPU启用
        copy_paste=0.0 if device == 'cpu' else 0.5,  # CPU禁用copy-paste，GPU启用
        save=True,
        save_period=10,
        cache=False if device == 'cpu' else True,  # CPU禁用缓存，GPU启用
        name=f'yolov8n_160_{"gpu" if device == "cuda" else "cpu"}',
        exist_ok=True,
        amp=True if device == 'cuda' else False,  # GPU启用自动混合精度
        half=False,  # 禁用半精度（边缘设备可能不支持）
        deterministic=False,
        verbose=True
    )
    
    print("训练完成!")
    print(f"训练设备: {device}")
    print(f"训练参数: 批量大小={batch_size}, workers={workers}")
    return model

def export_to_onnx(model):
    """导出模型为ONNX格式"""
    print("导出模型为ONNX格式...")
    
    # 导出ONNX
    success = model.export(
        format='onnx',
        imgsz=160,
        opset=12,
        simplify=True,
        dynamic=False,  # 使用静态输入以优化性能
        half=False,     # 不使用半精度（边缘设备可能不支持）
        int8=False      # 不使用int8量化（可后续添加）
    )
    
    if success:
        print("ONNX模型导出成功!")
        # 找到导出的ONNX文件
        onnx_files = list(Path(".").glob("*.onnx"))
        if onnx_files:
            print(f"导出的ONNX文件: {onnx_files[0]}")
            return str(onnx_files[0])
    else:
        print("ONNX导出失败!")
    
    return None

def main():
    """主函数"""
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='YOLOv8训练脚本 - 针对边缘设备优化')
    parser.add_argument('--gpu', action='store_true', help='使用GPU进行训练（如果可用）')
    parser.add_argument('--cpu', action='store_true', help='强制使用CPU进行训练')
    parser.add_argument('--epochs', type=int, default=50, help='训练周期数（默认: 50）')
    parser.add_argument('--batch', type=int, default=None, help='批量大小（默认: 根据设备自动调整）')
    args = parser.parse_args()
    
    print("=" * 60)
    print("YOLOv8训练脚本 - 针对边缘设备优化")
    print("=" * 60)
    
    # 确定是否使用GPU
    use_gpu = args.gpu and not args.cpu
    if args.cpu:
        print("强制使用CPU训练")
    elif use_gpu:
        print("尝试使用GPU训练（如果可用）")
    else:
        print("使用默认设置（自动检测设备）")
    
    # 1. 准备数据集
    print("\n1. 准备数据集...")
    dataset_dir, class_mapping = prepare_dataset()
    
    # 2. 创建配置文件
    print("\n2. 创建配置文件...")
    data_yaml_path = create_data_yaml(dataset_dir, class_mapping)
    
    # 3. 训练模型
    print("\n3. 训练模型...")
    model = train_yolo_model(data_yaml_path, use_gpu=use_gpu)
    
    # 4. 导出ONNX
    print("\n4. 导出ONNX模型...")
    onnx_path = export_to_onnx(model)
    
    if onnx_path:
        print(f"\n✅ 训练完成! ONNX模型已保存: {onnx_path}")
        print(f"   输入尺寸: 160x160")
        print(f"   类别数量: {len(class_mapping)}")
        print(f"   针对边缘设备优化: CPU X1 1GHz, 0.37GB RAM")
        
        # 检查训练设备
        gpu_available, _ = check_gpu_availability()
        if use_gpu and gpu_available:
            print(f"   训练设备: GPU (加速训练)")
        else:
            print(f"   训练设备: CPU (兼容性模式)")
    else:
        print("\n❌ 训练失败!")
    
    # 5. 复制ONNX文件到C++目录
    if onnx_path and Path(onnx_path).exists():
        dest_path = Path("C++") / "best.onnx"
        shutil.copy2(onnx_path, dest_path)
        print(f"✅ ONNX模型已复制到: {dest_path}")

if __name__ == "__main__":
    main()
