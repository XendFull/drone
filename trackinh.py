"""School video/webcam demo: manual motion detection plus photo classification.

Keep the camera still and the view clear during automatic background setup.
Run this file, or Run All in cnn.ipynb. Q/Esc or closing the window stops it.
X/Y mark each moving region's box center in the resized preview's pixels.
The top-left is (0, 0); X increases rightward and Y increases downward.
"""
from pathlib import Path
import time
import cv2
import numpy as np

TRACKER_VERSION = '3.2'
# Filter small specks and weak lighting changes in the 640-pixel-wide preview.
MIN_AREA = 2000
MIN_REGION_WIDTH = 50
MIN_REGION_HEIGHT = 40
CONFIRM_FRAMES = 3  # A region must persist before it gets a box.
CHANGE_THRESHOLD = 25
WARMUP_FRAMES = 30
LEARNING_RATE = 0.02
CLASSIFY_INTERVAL = 0.4
MAX_REGIONS = 2  # Maximum boxes displayed and classified per frame.
LABEL_LIFETIME = 0.8


class MotionDetector:
    def __init__(self, warmup_frames=WARMUP_FRAMES, min_area=MIN_AREA,
                 min_width=MIN_REGION_WIDTH, min_height=MIN_REGION_HEIGHT,
                 confirm_frames=CONFIRM_FRAMES, min_long_side=0):
        self.warmup_frames = warmup_frames
        self.min_area = min_area
        self.min_width = min_width
        self.min_height = min_height
        self.confirm_frames = confirm_frames
        self.min_long_side = min_long_side
        self.background = None
        self.frame_count = 0
        self.regions = []

    def update(self, frame):
        gray = cv2.GaussianBlur(frame.mean(axis=2).astype(np.float32), (3, 3), 0)
        self.frame_count += 1
        if self.background is None:
            self.background = gray.copy()
        if self.frame_count <= self.warmup_frames:
            self.background += (gray - self.background) / self.frame_count
            return np.zeros(gray.shape, np.uint8), []
        change = gray - self.background
        change -= np.median(change)
        mask = (np.abs(change) > CHANGE_THRESHOLD).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for contour in contours:
            if cv2.contourArea(contour) <= self.min_area:
                continue
            box = cv2.boundingRect(contour)
            if (box[2] >= self.min_width and box[3] >= self.min_height
                    and max(box[2:]) >= self.min_long_side):
                boxes.append(box)
        boxes.sort(key=lambda box: box[2] * box[3], reverse=True)
        # Suppress one-frame flashes, keeping separate counts for each region.
        previous = list(self.regions)
        current = []
        for box in boxes:
            confirmations = 1
            if previous:
                index = max(range(len(previous)), key=lambda i: overlap(box, previous[i][0]))
                if overlap(box, previous[index][0]) >= 0.2:
                    _, count = previous.pop(index)
                    confirmations = min(self.confirm_frames, count + 1)
            current.append((box, confirmations))
        self.regions = current
        boxes = [box for box, count in current if count >= self.confirm_frames]
        self.background += LEARNING_RATE * (gray - self.background)
        return mask, boxes


def crop_region(frame, box):
    x, y, width, height = box
    pad = max(8, round(0.15 * max(width, height)))
    left, top = max(0, x-pad), max(0, y-pad)
    right, bottom = min(frame.shape[1], x+width+pad), min(frame.shape[0], y+height+pad)
    return frame[top:bottom, left:right].copy()


def overlap(first, second):
    x, y, w, h = first
    a, b, c, d = second
    intersection = max(0, min(x+w, a+c)-max(x, a)) * max(0, min(y+h, b+d)-max(y, b))
    return intersection / (w*h+c*d-intersection)


def load_classifier(model_path=None):
    from cnn import load_model, predict_frames
    model, transform, class_names = load_model(model_path)
    return lambda frames: predict_frames(frames, model, transform, class_names)


def choose_video():
    import tkinter as tk
    from tkinter import filedialog
    root = tk.Tk()
    root.withdraw()
    video_path = filedialog.askopenfilename(parent=root, title='Choose a video',
        filetypes=[('Video files', '*.mp4 *.avi *.mov *.mkv *.webm *.m4v'), ('All files', '*.*')])
    root.destroy()
    return video_path


def run_source(source, classify_frames=None, video=False, inline=False, target_label=None):
    if classify_frames is None:
        classify_frames = load_classifier()
    if inline:
        from IPython.display import display, Image as DisplayImage
        print('Opening camera. The live feed appears below; interrupt this cell to stop.', flush=True)
        initial = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(initial, 'Opening camera...', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        preview = display(DisplayImage(data=cv2.imencode('.jpg', initial)[1].tobytes()), display_id=True)
    camera = cv2.VideoCapture(str(source)) if video else cv2.VideoCapture(source, cv2.CAP_MSMF)
    fps = camera.get(cv2.CAP_PROP_FPS) if video else 30.0
    detector_options = ({'min_area': 800, 'min_width': 12, 'min_height': 12,
                         'min_long_side': 50, 'confirm_frames': 2}
                        if target_label == 'glider' else {})
    detector = MotionDetector(warmup_frames=1 if video else WARMUP_FRAMES,
                              **detector_options)
    frame_number = 0
    cached_labels = []
    last_classified = float('-inf')
    window = 'Video motion + CNN | Q/Esc: quit' if video else 'Motion + CNN | Q/Esc: quit'
    try:
        while True:
            frame_started = time.monotonic()
            success, frame = camera.read()
            if not success:
                break
            height, width = frame.shape[:2]
            frame = cv2.resize(frame, (640, max(1, round(height * 640 / width))))
            _, boxes = detector.update(frame)
            boxes = boxes[:MAX_REGIONS]
            now = frame_number / fps if video else time.monotonic()
            frame_number += 1
            if detector.frame_count <= detector.warmup_frames:
                message = f'Clear view: learning background {detector.frame_count}/{detector.warmup_frames}'
            else:
                message = f'v{TRACKER_VERSION} | Regions: {len(boxes)}/{MAX_REGIONS} | Keep camera still | Q/Esc: quit'
                if not boxes:
                    cached_labels = []
                elif now - last_classified >= CLASSIFY_INTERVAL:
                    selected = boxes[:MAX_REGIONS]
                    results = classify_frames([crop_region(frame, box) for box in selected])
                    cached_labels = list(zip(selected, results))
                    last_classified = now if video else time.monotonic()
                for box in boxes:
                    x, y, w, h = box
                    center_x = x + w // 2
                    center_y = y + h // 2
                    label, score = 'Checking...', None
                    label_time = now if video else time.monotonic()
                    if label_time - last_classified <= LABEL_LIFETIME:
                        matches = [(overlap(box, old_box), result) for old_box, result in cached_labels]
                        if matches:
                            similarity, result = max(matches, key=lambda match: match[0])
                            if similarity >= 0.2:
                                label, score = result
                    if target_label is not None and label != target_label:
                        continue
                    color = ((0, 165, 255) if score is None or label == 'uncertain'
                             else (0, 0, 255) if label == 'missiles' else (0, 220, 0))
                    text = label if score is None else f'{label} | score {score:.0%}'
                    text += f' | X={center_x} Y={center_y}'
                    cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                    cv2.drawMarker(frame, (center_x, center_y), color,
                                   cv2.MARKER_CROSS, 14, 2)
                    text_width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0][0]
                    text_x = max(0, min(x, frame.shape[1] - text_width - 5))
                    cv2.putText(frame, text, (text_x, max(45, y-7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(frame, message, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
            coordinates_note = f'X/Y: box center in {frame.shape[1]}x{frame.shape[0]} preview | (0,0): top-left'
            cv2.putText(frame, coordinates_note, (10, frame.shape[0]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            if inline:
                encoded = cv2.imencode('.jpg', frame)[1]
                preview.update(DisplayImage(data=encoded.tobytes()))
                time.sleep(0.02)
            else:
                cv2.imshow(window, frame)
                delay = max(1, round(1000 / fps - 1000 * (time.monotonic() - frame_started))) if video else 1
                if cv2.waitKey(delay) & 0xFF in (ord('q'), 27):
                    break
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
    finally:
        camera.release()
        if not inline:
            cv2.destroyAllWindows()


def run_camera(classify_frames=None, camera_index=0, inline=False, target_label=None):
    return run_source(camera_index, classify_frames, video=False, inline=inline, target_label=target_label)


def run_video(video_path, classify_frames=None):
    return run_source(video_path, classify_frames, video=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Motion + CNN on a video or webcam")
    parser.add_argument("video", nargs="?", help="Path to a video file; omit to use webcam")
    parser.add_argument("--glider", action="store_true", help="Use the render-trained Glide350 prototype model")
    args = parser.parse_args()
    if args.glider:
        classifier = load_classifier(Path(__file__).with_name('glider_model.pth'))
        run_source(args.video if args.video else 0, classifier, video=bool(args.video), target_label='glider')
    else:
        run_video(args.video) if args.video else run_camera()
