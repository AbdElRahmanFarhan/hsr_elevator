import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
from torchvision.datasets import ImageFolder
from sklearn.model_selection import train_test_split
import numpy as np

class ElevatorButtonClassifier(nn.Module):
    def __init__(self):
        super(ElevatorButtonClassifier, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=3)
        self.bn1 = nn.BatchNorm2d(16)
        self.pool1 = nn.MaxPool2d(2,2)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=3)
        self.bn2 = nn.BatchNorm2d(32)
        self.pool2 = nn.MaxPool2d(2,2)
        self.conv3 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.bn3 = nn.BatchNorm2d(64)
        self.fc1 = nn.Linear(64 * 5 * 5, 16)
        self.dropout = nn.Dropout(0.2)
        self.fc2 = nn.Linear(16, 7)
        
    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.pool1(x)
        x = nn.functional.relu(x)
        x = self.conv2(x)
        x = self.bn2(x)
        x = self.pool2(x)
        x = nn.functional.relu(x)
        x = self.bn3(self.conv3(x))
        x = nn.functional.relu(x)
        x = x = torch.flatten(x, 1)
        x = nn.functional.relu(self.fc1(x))
        x = self.dropout(x)
        x = nn.functional.relu(self.fc2(x))
        return nn.functional.softmax(x, dim=1)