import torch
import torch.nn as nn
from randla_score_model import RandLANet
from pyg_randla_score_model import PyGRandLANet
import pickle as pkl
from torch.utils.data import DataLoader
from rgbd_mask_score_dataset import RGBDDataset
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm
import os

if __name__ == '__main__':
    log_dir = "run_maskscore/rgbd_score_training"
    os.makedirs(os.path.join(log_dir, "checkpoints"), exist_ok=True)
    writer = SummaryWriter(log_dir=log_dir)
    resume_training = False
    BS = 4  # Batch size
    total_epochs = 100
    last_epoch = 0

    data_dict = {"train": ["/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_seat_belt/dp_dataset_seg-187-9/train.pkl",
                                "/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_striker/dp_dataset_seg-95-4/train.pkl"],
                "test": ["/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_seat_belt/dp_dataset_seg-187-9/test.pkl",
                        "/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_striker/dp_dataset_seg-95-4/test.pkl"],}
    data_dict = {"train":[], "test":[]}
    files_train = os.listdir("mask_score/scoredds_pklfiles/train")
    files_test = os.listdir("mask_score/scoredds_pklfiles/test")
    for file in files_train:
        data_dict['train'].append(os.path.join("mask_score/scoredds_pklfiles/train", file))
    for file in files_test:
        data_dict['test'].append(os.path.join("mask_score/scoredds_pklfiles/test", file))
    train_dataset = RGBDDataset(data_files=data_dict['train'])
    test_dataset = RGBDDataset(data_files=data_dict['test'])
    train_loader = DataLoader(train_dataset, batch_size=BS, shuffle=True, num_workers=8, pin_memory=True)
    test_loader = DataLoader(test_dataset, batch_size=BS, shuffle=False, num_workers=8, pin_memory=True)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    d_in = 7  # RGB + Depth + Mask

    model = RandLANet(d_in, 10, 16, 4, device)

    model.train()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.MSELoss()
    lr_scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=total_epochs, eta_min=0.00001)
    best_val_loss = float('inf')
    if resume_training:
        model.load_state_dict(torch.load(os.path.join(log_dir, 'checkpoints', 'last_model.pth'))["model"])
        optimizer.load_state_dict(torch.load(os.path.join(log_dir, 'checkpoints', 'last_model.pth'))["optimizer"])
        lr_scheduler.load_state_dict(torch.load(os.path.join(log_dir, 'checkpoints', 'last_model.pth'))["lr_scheduler"])
        last_epoch = torch.load(os.path.join(log_dir, 'checkpoints', 'last_model.pth'))["epoch"]
        best_val_loss = torch.load(os.path.join(log_dir, 'checkpoints', 'best_model.pth'))["val_loss"]
        print(f"Resuming training from epoch {last_epoch} with best validation loss: {best_val_loss:.4f}")
    else:
        print("Starting training from scratch.")
    for epoch in range(last_epoch, total_epochs):
        model.train()
        pbar = tqdm(train_loader, desc="Training", leave=False)
        for batch in pbar:
            optimizer.zero_grad()
            inputs, targets = batch
            inputs = inputs.to(device)
            targets = targets.to(device)
            # Forward pass
            outputs = model(inputs)
            loss = criterion(torch.sigmoid(outputs), targets)
            loss.backward()
            optimizer.step()
            lr_scheduler.step()
            pbar.set_description(f"Epoch {epoch+1}/{total_epochs} - Loss: {loss.item():.4f}")
            writer.add_scalar("Loss/train", loss.item(), epoch * len(train_loader) + pbar.n)

        # Validation
        if epoch % 10 == 0 or epoch == total_epochs - 1:
            print(f"Validation at epoch {epoch+1}/{total_epochs}")
            model.eval()
            val_loss = 0.0
            with torch.no_grad():
                pbar_val = tqdm(test_loader, desc="Validation", unit="batch", leave=False)
                for batch in pbar_val:
                    inputs, targets = batch
                    inputs = inputs.to(device)
                    targets = targets.to(device)
                    outputs = model(inputs)
                    loss = criterion(torch.sigmoid(outputs), targets)
                    val_loss += loss.item()
                    pbar_val.set_description(f"Validation Epoch {epoch+1}/{total_epochs} - Loss: {loss.item():.4f}")
                    writer.add_scalar("Loss/val", loss.item(), epoch * len(test_loader) + pbar_val.n)
            print(f"Validation Loss: {val_loss:.4f}")

            if val_loss < best_val_loss:
                best_val_loss = val_loss
                torch.save({
                    "model": model.state_dict(),
                    "optimizer": optimizer.state_dict(),
                    "lr_scheduler": lr_scheduler.state_dict(),
                    "epoch": epoch,
                    "val_loss": best_val_loss
                }, os.path.join(log_dir, 'checkpoints', 'best_model.pth'))
                print(f"New best model saved at epoch {epoch+1} with validation loss: {best_val_loss:.4f}")
                print(f"Best model saved at epoch {epoch+1} with validation loss: {best_val_loss:.4f}")
        # Save the last model state
        torch.save({
            "model": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "lr_scheduler": lr_scheduler.state_dict(),
            "epoch": epoch,
            "val_loss": float('inf')
        }, os.path.join(log_dir, 'checkpoints', 'last_model.pth'))
        # torch.save(model.state_dict(), os.path.join(log_dir, 'checkpoints', 'last_model.pth'))
