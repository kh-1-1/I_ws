
"""
ObsPointNet is a neural network structure of DUNE model. It maps each obstacle point to the latent distance feature mu.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

NeuPAN planner is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

NeuPAN planner is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with NeuPAN planner. If not, see <https://www.gnu.org/licenses/>.
"""
import torch.nn as nn
import torch

class ObsPointNet(nn.Module):
    """
    ObsPointNet 是一个神经网络模块，用于将障碍物点映射到潜在距离特征空间。
    
    参数:
        input_dim (int): 输入维度，默认为2，表示2D坐标点
        output_dim (int): 输出维度，默认为4，表示潜在特征维度
    """
    def __init__(self, input_dim: int = 2, output_dim: int = 4) -> None:
        super(ObsPointNet, self).__init__()
        
        # 隐藏层维度
        hidden_dim = 32
        
        # 构建多层感知机(MLP)网络
        # 网络结构: 输入层 -> (Linear->LayerNorm->Tanh/ReLU) x 5 -> 输出层
        self.MLP = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),    # 第一层: 输入维度 -> 隐藏维度
            nn.LayerNorm(hidden_dim),            # 层归一化
            nn.Tanh(),                           # 激活函数
            
            nn.Linear(hidden_dim, hidden_dim),   # 第二层
            nn.ReLU(),                           # ReLU激活
            
            nn.Linear(hidden_dim, hidden_dim),   # 第三层
            nn.LayerNorm(hidden_dim),
            nn.Tanh(),
            
            nn.Linear(hidden_dim, hidden_dim),   # 第四层
            nn.ReLU(),
            
            nn.Linear(hidden_dim, hidden_dim),   # 第五层
            nn.LayerNorm(hidden_dim),
            nn.Tanh(),
            
            nn.Linear(hidden_dim, output_dim),   # 输出层: 隐藏维度 -> 输出维度
            nn.ReLU(),                           # 最终激活函数
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        前向传播函数
        
        参数:
            x (torch.Tensor): 输入张量，形状为 [..., input_dim]
            
        返回:
            torch.Tensor: 输出张量，形状为 [..., output_dim]
        """
        return self.MLP(x)