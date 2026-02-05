import cv2
from ultralytics import YOLO

DEVICE = "/dev/video4"   # ✅ external HD USB camera
MODEL  = "yolov8s.pt"

def main():
    model = YOLO(MODEL)

    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera device {DEVICE}")

    print(f"Running YOLO on {DEVICE}. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed.")
            break

        results = model.predict(frame, imgsz=640, conf=0.25, verbose=False)
        annotated = results[0].plot()

        cv2.imshow("YOLO Live (External Cam)", annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
