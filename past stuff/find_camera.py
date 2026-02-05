import cv2

# Test camera indices from 0 to 5
for i in range(6):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"Camera {i}: Working")
            cv2.imshow(f'Camera {i}', frame)
            cv2.waitKey(2000)  # Show for 2 seconds
            cap.release()
            cv2.destroyAllWindows()
        else:
            print(f"Camera {i}: Opened but can't read")
            cap.release()
    else:
        print(f"Camera {i}: Not available")