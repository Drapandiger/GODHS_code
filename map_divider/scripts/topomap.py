import yaml
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

# 读取YAML文件
with open('submaps.yaml', 'r') as file:
    data = yaml.safe_load(file)

# 创建一个空的无向图
G = nx.Graph()

# 遍历所有区域，添加节点和连通关系
for area in data['areas']:
    room_name = area['name']
    
    # 根据区域级别设置节点属性
    if area.get('level', '') == 'high':
        G.add_node(room_name, level='high', color='red', size=1000)  # 走廊等高等级区域
    else:
        G.add_node(room_name, level='normal', color='blue', size=700)  # 普通区域
    
    # 添加与其他房间的连通关系
    if 'connections' in area:
        for connection in area['connections']:
            connected_room = connection['to']
            G.add_edge(room_name, connected_room)

# 获取每个节点的颜色和大小
colors = [G.nodes[node]['color'] for node in G.nodes()]
sizes = [G.nodes[node]['size'] for node in G.nodes()]

# 使用spring布局计算节点位置
pos = nx.spring_layout(G)

# 创建绘图区域
fig, ax = plt.subplots(figsize=(10, 7))

# 绘制拓扑图的节点和边（不绘制标签和节点颜色）
nx.draw(G, pos, ax=ax, node_size=sizes, edge_color='gray', with_labels=False)

# 为每个房间的节点添加平面图
for node in G.nodes():
    try:
        # 读取每个房间的平面图
        img_path = f"{node}.png"
        img = mpimg.imread(img_path)
        
        # 使用 OffsetImage 将平面图嵌入到节点位置
        imagebox = OffsetImage(img, zoom=0.15)  # zoom 用来调整图片大小
        ab = AnnotationBbox(imagebox, pos[node], frameon=False)
        ax.add_artist(ab)
    except FileNotFoundError:
        print(f"无法找到平面图: {img_path}")

# 添加节点标签
nx.draw_networkx_labels(G, pos, font_size=12, font_weight='bold', labels={node: node for node in G.nodes()})

# 设置标题
plt.title("Room Connectivity with Floor Plans", fontsize=15)

# 保存图形到本地
plt.savefig("room_connectivity_topology.png")  # 将图像保存为 PNG 文件
plt.savefig("room_connectivity_topology.svg")   # 保存为 SVG 文件

# 显示图形
plt.show()
