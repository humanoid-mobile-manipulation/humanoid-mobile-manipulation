import sys
import os
import pickle
import numpy as np
import cv2
import torch
import shutil
from ultralytics import YOLO
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QListWidget, QFileDialog, QTextEdit, QSizePolicy, QProgressBar, QInputDialog
)
from PyQt5.QtGui import QPixmap, QImage, QMouseEvent, QKeyEvent, QResizeEvent
from PyQt5.QtCore import Qt, QPoint, QDir
from datetime import datetime  
import cv2

block_range = [(0,360), (330,640)]

def block_img(img, blk_range):
    fill_color = (0,0,0)
    img[blk_range[0][0]:blk_range[0][1], blk_range[1][0]:blk_range[1][1]] = fill_color
    return img

# Load YOLO model
# head_yolo_model_path = '/home/qiang/FoundationPose/kuavo_seg/yolo_seg/yolo_models/bracket_320/weights/best.pt'
head_yolo_model_path = 'runs/segment/train2/weights/best.pt'
head_yolo_model = YOLO(head_yolo_model_path, verbose=False).to(torch.device("cuda:0"))

class CustomLogger:
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, message):
        self.text_widget.append(message.strip())
        self.text_widget.ensureCursorVisible()

    def flush(self):
        pass

def select_image_or_folder():
    dialog = QFileDialog()
    dialog.setDirectory("./")  # Initial directory.
    dialog.setWindowTitle("Select Image or Folder")
    dialog.setFileMode(QFileDialog.ExistingFiles)  # Allow multiple image files.
    dialog.setOption(QFileDialog.DontUseNativeDialog, True)  # Use Qt dialog for folder/file selection.
    dialog.setNameFilter("Images (*.jpg *.jpeg *.png)")  # Show image files.
    dialog.setFilter(QDir.AllDirs | QDir.Files)  # Show folders and files.

    if dialog.exec_() == QFileDialog.Accepted:
        selected_files = dialog.selectedFiles()
        if selected_files:
            parent_dir = os.path.dirname(selected_files[0])
            print("Selected top folder:", parent_dir)
            return parent_dir
    return None

class SegMaskScorerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.StrongFocus)

        self.img_files = []
        self.current_file_index = 0
        self.current_frame_index = 0
        self.data = None
        self.data_path = None
        self.masks = None
        self.resized_masks = None
        self.mask_scores = None
        self.selected_mask_index = None
        self.image_size = None
        self.mask_overlay = False
        self.output_folder = None

        self.init_ui()
        sys.stdout = CustomLogger(self.log_text)

    def init_ui(self):
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()

        self.rules_label = QLabel(
            "Scoring guidelines:\n"
            "- Reachability: assign 0 if the end-effector cannot approach the target.\n"
            "- Occlusion: lower the score if foreground objects block the arm path.\n"
            "- Boundary proximity: lower the score for targets close to tray or bin edges.\n"
            "- Clutter: lower the score for stacked or tightly packed targets.\n"
            "- Mask quality: lower the score for noisy or merged segmentation masks.\n"
            "- IK difficulty: lower the score when the target location is hard for IK."
        )
        self.rules_label.setStyleSheet("color: darkblue; font-size: 12px;")
        left_layout.addWidget(self.rules_label)

        self.image_label = QLabel("No Image Loaded")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.mousePressEvent = self.image_clicked
        left_layout.addWidget(self.image_label, 4)

        nav_layout = QHBoxLayout()
        for text, handler in [("W", self.prev_file), ("S", self.next_file),
                              ("1 (YOLO)", self.run_yolo_and_display_masks),
                              ("Enter (Save)", self.save_current_frame)]:
            btn = QPushButton(text)
            btn.clicked.connect(handler)
            nav_layout.addWidget(btn)
        left_layout.addLayout(nav_layout)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: black; color: lightgreen;")
        left_layout.addWidget(self.log_text, 2)

        right_layout = QVBoxLayout()
        self.file_list = QListWidget()
        self.file_list.itemClicked.connect(self.load_selected_file)
        self.file_list.setStyleSheet("""
                                     QListWidget::item:selected {
                                         background-color: #0078D7;
                                         color: white;
                                     }
                                     """)
        right_layout.addWidget(self.file_list)

        btn_select_folder = QPushButton("Select Image Folder")
        btn_select_folder.clicked.connect(self.select_folder)
        right_layout.addWidget(btn_select_folder)

        btn_output_folder = QPushButton("Select Output Folder")
        btn_output_folder.clicked.connect(self.select_output_folder)
        right_layout.addWidget(btn_output_folder)

        main_layout.addLayout(left_layout, 3)
        main_layout.addLayout(right_layout, 1)

        self.setLayout(main_layout)
        self.setWindowTitle("SegMaskScorer")
        self.resize(1100, 700)

    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        if key == Qt.Key_W:
            self.prev_file()
        elif key == Qt.Key_S:
            self.next_file()
        elif key == Qt.Key_1:
            self.run_yolo_and_display_masks()
        elif key in [Qt.Key_Return, Qt.Key_Enter]:
            self.save_current_frame()

    def select_folder(self):
        # folder_path = QFileDialog.getExistingDirectory(self, "Select Image Folder")
        folder_path = select_image_or_folder()
        if folder_path is not None and folder_path:
            self.img_files = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.jpg') or f.endswith('.png')]
            self.img_files.sort()
            self.file_list.clear()
            self.file_list.addItems([os.path.basename(f) for f in self.img_files])
            if self.img_files:
                self.load_file(0)

    def select_output_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Output Folder")
        if folder:
            self.output_folder = folder
            print(f"Selected output folder: {folder}.")

    def load_selected_file(self, item):
        index = self.file_list.currentRow()
        self.load_file(index)

    def load_file(self, index):
        self.current_file_index = index
        self.file_list.setCurrentRow(index)
        self.current_frame_index = 0
        
        self.data = cv2.imread(self.img_files[index])
        self.data_path = self.img_files[index]
        
        print(f"Loaded: {os.path.basename(self.img_files[index])}.")
        self.masks = None
        self.resized_masks = None
        self.mask_scores = None
        self.selected_mask_index = None
        self.mask_overlay = False
        self.display_image()

    def display_image(self, mask_overlay=False):
        img = self.data
        img = cv2.cvtColor(np.array(img, dtype=np.uint8), cv2.COLOR_BGR2RGB)
        self.image_size = (img.shape[1], img.shape[0])

        if mask_overlay and self.resized_masks is not None:
            overlay = img.copy()
            for i, mask in enumerate(self.resized_masks):


                color = (0, 255, 0) if self.mask_scores[i] is not None else (255, 255, 255) # (128, 128, 128)
                color_mask = np.zeros_like(overlay)
                for c in range(3):
                    color_mask[:, :, c] = mask * color[c]
                overlay = cv2.addWeighted(overlay, 1.0, color_mask, 0.7, 0)

                D_M = cv2.moments(mask.astype(np.uint8))
                if D_M["m00"] != 0:
                    DM_cx = int(D_M["m10"] / D_M["m00"])
                    DM_cy = int(D_M["m01"] / D_M["m00"])
                    cv2.putText(overlay, str(i), (DM_cx - 10, DM_cy), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (255, 0, 0), 2, cv2.LINE_AA)

                if self.mask_scores[i] is not None:
                    score_text = f"{self.mask_scores[i]:.2f}"
                    M = cv2.moments(mask.astype(np.uint8))
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.putText(overlay, score_text, (cx - 10, cy), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.4, (0, 0, 255), 1, cv2.LINE_AA)
                        
            img = overlay

        qimg = QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def run_yolo_and_display_masks(self):
        head_img = self.data
        if head_img is None:
            print("No image loaded. Load an image first.")
            return
        yolo_head_img = head_img.copy()
        # yolo_head_img = block_img(yolo_head_img, blk_range=block_range)
        # results = head_yolo_model.track(yolo_head_img, conf=0.1, iou=0.38, persist=False, show=False)
        results = head_yolo_model.predict(yolo_head_img, conf=0.7)
        result = results[0]
        self.masks = result.masks.data.cpu().numpy()
        self.resized_masks = [cv2.resize(mask.astype(np.uint8), (head_img.shape[1], head_img.shape[0]), interpolation=cv2.INTER_NEAREST)
                              for mask in self.masks]
        self.mask_scores = [None] * len(self.resized_masks)
        self.selected_mask_index = None
        self.mask_overlay = True
        print(f"YOLO detected {len(self.masks)} masks.")
        self.display_image(mask_overlay=True)

    def image_clicked(self, event: QMouseEvent):
        if self.resized_masks is None:
            print("No masks available. Run YOLO first; skip this image if no mask is detected.")
            return

        label_size = self.image_label.size()
        pixmap_size = self.image_label.pixmap().size()
        offset_x = (label_size.width() - pixmap_size.width()) // 2
        offset_y = (label_size.height() - pixmap_size.height()) // 2
        click_pos = event.pos() - QPoint(offset_x, offset_y)
        x = int(click_pos.x() * self.image_size[0] / pixmap_size.width())
        y = int(click_pos.y() * self.image_size[1] / pixmap_size.height())

        if x < 0 or y < 0 or x >= self.image_size[0] or y >= self.image_size[1]:
            print("Clicked outside image bounds.")
            return

        for i, mask in enumerate(self.resized_masks):
            if y < mask.shape[0] and x < mask.shape[1] and mask[y, x] == 1:
                self.selected_mask_index = i
                score, ok = QInputDialog.getDouble(self, "Set Score", f"Set score for mask {i} (0.0~1.0):", decimals=2, min=0.0, max=1.0)
                if ok:
                    self.mask_scores[i] = float(score)
                    print(f"Mask {i} scored: {score:.2f}.")
                self.display_image(mask_overlay=True)
                return
        print("Clicked point is not inside any mask.")

    def save_current_frame(self):
        if self.output_folder is None:
            print("No output folder selected.")
            return
        if self.data is None or self.resized_masks is None or self.mask_scores is None or self.data_path is None:
            print("No data or masks to save.")
            return
        img = self.data
        save_dict = {
            'head_img': img,
            'masks': self.resized_masks,
            'scores': self.mask_scores
        }
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # save_path = os.path.join(self.output_folder, f"scored_frame_{timestamp}.pkl")
        save_path = os.path.join(self.output_folder,self.data_path.split('/')[-1].split('.')[0] + '_scored.pkl')
        
        if None in self.mask_scores:
            print("Warning: not all masks have been scored. Score every mask before saving.")
            return
        
        with open(save_path, 'wb') as f:
            pickle.dump(save_dict, f)
        print(f"Saved frame to {save_path}.")

    def prev_file(self):
        if self.current_file_index > 0:
            self.load_file(self.current_file_index - 1)

    def next_file(self):
        if self.current_file_index < len(self.img_files) - 1:
            self.load_file(self.current_file_index + 1)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = SegMaskScorerApp()
    viewer.show()
    sys.exit(app.exec_())
