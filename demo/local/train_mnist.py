"""
Train a small LeNet on MNIST and export it to ONNX for the demo's fallback
classifier. Only this script needs torch; the demo runs the ONNX file.

    pip install torch torchvision onnx onnxscript
    python3 train_mnist.py        # writes outputs/mnist_lenet.onnx
"""
import argparse
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader


class LeNet(nn.Module):
    """LeNet-5 style, about 65k parameters."""
    def __init__(self, num_classes=10):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 16, kernel_size=5, padding=2)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, padding=0)
        self.fc1   = nn.Linear(32 * 5 * 5, 64)
        self.fc2   = nn.Linear(64, num_classes)

    def forward(self, x):
        x = F.max_pool2d(F.relu(self.conv1(x)), 2)
        x = F.max_pool2d(F.relu(self.conv2(x)), 2)
        x = x.flatten(1)
        x = F.relu(self.fc1(x))
        return self.fc2(x)


def train(epochs, batch_size, lr, data_dir, device=None):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    print(f"device: {device}")

    tfm = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,)),
    ])
    train_ds = datasets.MNIST(data_dir, train=True,  download=True, transform=tfm)
    test_ds  = datasets.MNIST(data_dir, train=False, download=True, transform=tfm)
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    test_loader  = DataLoader(test_ds,  batch_size=512, shuffle=False)

    model = LeNet().to(device)
    opt = optim.Adam(model.parameters(), lr=lr)

    for epoch in range(1, epochs + 1):
        model.train()
        running = 0.0
        for i, (x, y) in enumerate(train_loader):
            x, y = x.to(device), y.to(device)
            opt.zero_grad()
            loss = F.cross_entropy(model(x), y)
            loss.backward(); opt.step()
            running += loss.item()
            if (i + 1) % 100 == 0:
                print(f"  epoch {epoch}  batch {i+1}/{len(train_loader)}  "
                      f"loss {running/100:.4f}")
                running = 0.0

        model.eval()
        correct = total = 0
        with torch.no_grad():
            for x, y in test_loader:
                x, y = x.to(device), y.to(device)
                correct += (model(x).argmax(1) == y).sum().item()
                total += y.numel()
        print(f"  epoch {epoch} test accuracy: {100*correct/total:.2f}%")

    return model.cpu().eval()


def export_onnx(model, out_path):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    dummy = torch.randn(1, 1, 28, 28)
    # no dynamic_axes: the demo sends one image at a time, and the newer
    # exporter warns about them
    torch.onnx.export(
        model, dummy, str(out_path),
        input_names=["image"],
        output_names=["logits"],
        opset_version=18,
    )
    print(f"wrote {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=3)
    ap.add_argument("--batch-size", type=int, default=128)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--out", default="outputs/mnist_lenet.onnx",
                    help="ONNX output path (default: outputs/mnist_lenet.onnx)")
    ap.add_argument("--data-dir", default="outputs/data",
                    help="MNIST cache dir (default: outputs/data)")
    args = ap.parse_args()

    Path(args.data_dir).mkdir(parents=True, exist_ok=True)
    model = train(args.epochs, args.batch_size, args.lr, args.data_dir)
    export_onnx(model, args.out)
    print("done. deploy this ONNX file with the demo; no torch needed at runtime.")


if __name__ == "__main__":
    main()
