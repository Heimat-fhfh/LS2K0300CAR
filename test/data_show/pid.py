import re
import matplotlib.pyplot as plt
from typing import List, Dict
import sys

def parse_data_from_text(text: str) -> List[Dict[str, float]]:
    """
    从文本中解析数据
    每行格式: Status - Target:2.000, Current:0.865, Output:0.252, Error:1.135
    """
    data = []
    pattern = r'Target:([\d.]+),\s*Current:([\d.]+),\s*Output:([\d.]+),\s*Error:([\d.]+)'
    
    for line in text.strip().split('\n'):
        match = re.search(pattern, line)
        if match:
            data_point = {
                'target': float(match.group(1)),
                'current': float(match.group(2)),
                'output': float(match.group(3)),
                'error': float(match.group(4))
            }
            data.append(data_point)
    
    return data

def read_data_from_file(filename: str) -> List[Dict[str, float]]:
    """从文件读取数据"""
    try:
        with open(filename, 'r') as file:
            content = file.read()
        return parse_data_from_text(content)
    except FileNotFoundError:
        print(f"错误: 找不到文件 '{filename}'")
        return []
    except Exception as e:
        print(f"读取文件时出错: {e}")
        return []

def plot_data(data: List[Dict[str, float]], title: str = "数据折线图"):
    """
    绘制数据的折线图
    """
    if not data:
        print("没有数据可绘制")
        return
    
    # 创建x轴数据（数据点序号）
    x = list(range(1, len(data) + 1))
    
    # 提取各个变量的值
    targets = [d['target'] for d in data]
    currents = [d['current'] for d in data]
    outputs = [d['output'] for d in data]
    errors = [d['error'] for d in data]
    
    # 创建图形
    plt.figure(figsize=(12, 8))
    
    # 绘制折线图
    plt.plot(x, targets, 'r-', label='Target', marker='o', linewidth=2)
    plt.plot(x, currents, 'b-', label='Current', marker='s', linewidth=2)
    plt.plot(x, outputs, 'g-', label='Output', marker='^', linewidth=2)
    plt.plot(x, errors, 'orange', label='Error', marker='d', linewidth=2, linestyle='--')
    
    # 设置图表属性
    plt.xlabel('数据点序号', fontsize=12)
    plt.ylabel('数值', fontsize=12)
    plt.title(title, fontsize=14)
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, alpha=0.3)
    
    # 设置y轴范围，留一些边距
    all_values = targets + currents + outputs + errors
    y_min, y_max = min(all_values), max(all_values)
    margin = (y_max - y_min) * 0.1
    plt.ylim(y_min - margin, y_max + margin)
    
    # 显示图表
    plt.tight_layout()
    plt.show()

def save_data_to_csv(data: List[Dict[str, float]], filename: str = "output.csv"):
    """将数据保存为CSV文件"""
    if not data:
        return
    
    import csv
    
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['target', 'current', 'output', 'error']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for d in data:
            writer.writerow(d)
    
    print(f"数据已保存到 {filename}")

def main():
    # 示例数据（直接放在脚本中）
    sample_data = """
Status - Target:2.000, Current:0.865, Output:0.252, Error:1.135
Status - Target:2.000, Current:1.054, Output:0.305, Error:0.946
Status - Target:2.000, Current:1.405, Output:0.315, Error:0.595
Status - Target:2.000, Current:1.550, Output:0.334, Error:0.450
Status - Target:2.000, Current:1.721, Output:0.340, Error:0.279
Status - Target:2.000, Current:1.712, Output:0.362, Error:0.288
Status - Target:2.000, Current:1.883, Output:0.351, Error:0.117
Status - Target:2.000, Current:1.883, Output:0.363, Error:0.117
"""
    
    print("=" * 50)
    print("数据折线图生成工具")
    print("=" * 50)
    print("\n请选择数据来源:")
    print("1. 使用示例数据")
    print("2. 从文件读取数据")
    print("3. 直接粘贴数据")
    
    choice = input("\n请输入选项 (1/2/3): ").strip()
    
    data = []
    
    if choice == '1':
        data = parse_data_from_text(sample_data)
        print("已加载示例数据")
        
    elif choice == '2':
        filename = input("请输入文件名: ").strip()
        if not filename:
            filename = "data.txt"
        data = read_data_from_file(filename)
        
    elif choice == '3':
        print("\n请粘贴数据（输入完成后，在新的一行输入 'END' 结束）:")
        lines = []
        while True:
            line = input()
            if line.strip().upper() == 'END':
                break
            lines.append(line)
        
        if lines:
            data = parse_data_from_text('\n'.join(lines))
            print(f"已解析 {len(data)} 条数据")
    
    if not data:
        print("没有有效数据，程序退出")
        return
    
    # 显示数据统计
    print(f"\n共解析 {len(data)} 条数据")
    print(f"数据范围: 第1条到第{len(data)}条")
    
    # 询问是否保存数据为CSV
    save_choice = input("\n是否将数据保存为CSV文件? (y/n): ").strip().lower()
    if save_choice == 'y':
        csv_filename = input("请输入CSV文件名 (默认: output.csv): ").strip()
        if not csv_filename:
            csv_filename = "output.csv"
        save_data_to_csv(data, csv_filename)
    
    # 询问是否自定义图表标题
    title = input("\n请输入图表标题 (直接回车使用默认标题): ").strip()
    if not title:
        title = "数据折线图"
    
    # 绘制折线图
    plot_data(data, title)

if __name__ == "__main__":
    # 检查是否安装了matplotlib
    try:
        import matplotlib
    except ImportError:
        print("错误: 需要安装 matplotlib 库")
        print("请运行: pip install matplotlib")
        sys.exit(1)
    
    main()