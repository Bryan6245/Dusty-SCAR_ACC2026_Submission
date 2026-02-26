#!/usr/bin/env python3
import csv, random
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

DATA_DIR = Path("/workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission/dusty_scar/dusty_scar/New_Routes/Route8/combo/route8_combo_v5")
LABELS = DATA_DIR / "labels.csv"
MODEL_OUT = Path("/workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission/dusty_scar/dusty_scar/New_Routes/Route8/models/route8_steer_v5.pt")

IMG_W, IMG_H = 160, 120
BATCH = 64
EPOCHS = 25
LR = 1e-3
VAL_FRAC = 0.15
SEED = 7

MAX_ANG = 1.5  # clamp steering labels

def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

class BCDataset(Dataset):
    def __init__(self, rows, train=True):
        self.rows = rows
        self.train = train

    def __len__(self):
        return len(self.rows)

    def augment(self, img, ang):
        # brightness
        if random.random() < 0.7:
            factor = random.uniform(0.6, 1.4)
            img = np.clip(img.astype(np.float32) * factor, 0, 255).astype(np.uint8)

        # small shift
        if random.random() < 0.7:
            dx = random.randint(-8, 8)
            dy = random.randint(-4, 4)
            M = np.float32([[1, 0, dx], [0, 1, dy]])
            img = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]), borderMode=cv2.BORDER_REPLICATE)

        return img, ang

    def __getitem__(self, i):
        rel_img, lin, ang, t, match_dt = self.rows[i]
        img_path = DATA_DIR / rel_img
        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)  # BGR
        if img is None:
            raise RuntimeError(f"Failed to read {img_path}")

        img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
        ang = float(np.clip(float(ang), -MAX_ANG, MAX_ANG))

        if self.train:
            img, ang = self.augment(img, ang)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        x = (img.astype(np.float32) / 255.0).transpose(2, 0, 1)  # C,H,W
        return torch.from_numpy(x), torch.tensor([ang], dtype=torch.float32)

class SmallCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.feat = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, 3, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, 3, stride=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64, 64), nn.ReLU(),
            nn.Linear(64, 1)
        )

    def forward(self, x):
        return self.head(self.feat(x))

def load_rows():
    rows = []
    with open(LABELS, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append((r["image"], float(r["linear_x"]), float(r["angular_z"]), float(r["t"]), float(r["match_dt"])))
    return rows

def main():
    set_seed(SEED)
    rows = load_rows()
    random.shuffle(rows)

    n_val = max(1, int(len(rows) * VAL_FRAC))
    val_rows = rows[:n_val]
    train_rows = rows[n_val:]

    train_ds = BCDataset(train_rows, train=True)
    val_ds = BCDataset(val_rows, train=False)

    # num_workers=0 is safest in containers
    train_dl = DataLoader(train_ds, batch_size=BATCH, shuffle=True, num_workers=0)
    val_dl = DataLoader(val_ds, batch_size=BATCH, shuffle=False, num_workers=0)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("Device:", device, "| train:", len(train_ds), "| val:", len(val_ds))

    model = SmallCNN().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=LR)
    loss_fn = nn.MSELoss()

    best_val = 1e9
    for ep in range(1, EPOCHS + 1):
        model.train()
        tr_losses = []
        for x, y in train_dl:
            x, y = x.to(device), y.to(device)
            pred = model(x)
            loss = loss_fn(pred, y)
            opt.zero_grad()
            loss.backward()
            opt.step()
            tr_losses.append(loss.item())

        model.eval()
        va_losses = []
        with torch.no_grad():
            for x, y in val_dl:
                x, y = x.to(device), y.to(device)
                pred = model(x)
                loss = loss_fn(pred, y)
                va_losses.append(loss.item())

        tr = float(np.mean(tr_losses)) if tr_losses else float("nan")
        va = float(np.mean(va_losses)) if va_losses else float("nan")
        print(f"Epoch {ep:02d} | train MSE={tr:.5f} | val MSE={va:.5f}")

        if va < best_val:
            best_val = va
            MODEL_OUT = Path("/workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission/dusty_scar/dusty_scar/New_Routes/Route8/models/route8_steer_v5.pt")
            torch.save({
                "model_state": model.state_dict(),
                "img_w": IMG_W, "img_h": IMG_H,
                "max_ang": MAX_ANG
            }, MODEL_OUT)
            print(f"✅ Saved best model: {MODEL_OUT} (val={best_val:.5f})")

    print("Done.")

if __name__ == "__main__":
    main()
