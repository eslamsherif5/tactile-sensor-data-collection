import time
import cv2

cap = cv2.VideoCapture('/dev/video3')
# Variable to keep track of frames processed
frame_count = 0

# Start time to calculate fps
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    size = (frame.shape[0]//4, frame.shape[1]//4)
    cv2.imshow('frame',cv2.resize(frame, size))
    # Increment frame count
    frame_count += 1
    
    # Calculate elapsed time
    elapsed_time = time.time() - start_time
    
    if elapsed_time >= 1.0:
        fps = frame_count / elapsed_time
        print(f"FPS: {fps:.2f}")
        print(frame.shape)

        # Reset variables for the next second
        frame_count = 0
        start_time = time.time()


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()