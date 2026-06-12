import onnx
model = onnx.load("yolo/upload/model/v8V3.onnx")

# 查看模型输入输出详情
print("=== 模型输入 ===")
for input in model.graph.input:
    print(f"名称: {input.name}")
    print(f"类型: {input.type}")
    print(f"维度: {input.type.tensor_type.shape.dim}")

print("\n=== 模型输出 ===")
for output in model.graph.output:
    print(f"名称: {output.name}")
    print(f"类型: {output.type}")
    print(f"维度: {output.type.tensor_type.shape.dim}")

# 检查是否有sigmoid/softmax节点
print("\n=== 查找激活函数节点 ===")
for node in model.graph.node:
    if node.op_type in ['Sigmoid', 'Softmax', 'Clip']:
        print(f"找到 {node.op_type} 节点")
        print(f"输入: {node.input}")
        print(f"输出: {node.output}")