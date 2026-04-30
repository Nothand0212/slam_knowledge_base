import matplotlib.pyplot as plt
import numpy as np

def load_txt(file_path):
    """
    加载 txt 文件，返回 numpy 数组，格式为 [ts, x, y, z, roll, pitch, yaw]
    """
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            tokens = line.strip().split()
            if len(tokens) == 7:
                data.append([float(tok) for tok in tokens])
    return np.array(data)  # shape: [N, 7]

def plot_data(data1, data2, labels=['x', 'y', 'z', 'roll', 'pitch', 'yaw']):
    """
    绘图比较两个数据源中每个维度随时间的变化
    """
    ts1 = data1[:, 0]
    ts2 = data2[:, 0]

    fig, axes = plt.subplots(3, 2, figsize=(12, 8))
    axes = axes.flatten()

    for i in range(6):
        axes[i].plot(ts1, data1[:, i + 1], label='gt', color='blue')
        axes[i].plot(ts2, data2[:, i + 1], label='target', color='red', linestyle='--')
        axes[i].set_title(labels[i])
        axes[i].set_xlabel('Timestamp')
        axes[i].set_ylabel(labels[i])
        axes[i].legend()
        axes[i].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 替换为你的文件路径
    file1_path = "data/rtk_pose.txt"
    file2_path = "data/cur_pose.txt"

    data1 = load_txt(file1_path)
    data1[:,4:] *= 180/np.pi
    data2 = load_txt(file2_path)
    data2[:,4:] *= 180/np.pi
    plot_data(data1, data2)
