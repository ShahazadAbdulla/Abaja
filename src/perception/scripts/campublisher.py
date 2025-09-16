import cv2

# #Open the /dev/video0 device (index 0)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

# Display a window to show the frames
cv2.namedWindow("Frame", cv2.WINDOW_AUTOSIZE)

while True:
    # Read a frame from the video device
    # ret is a boolean, true if the frame was read successfully
    # frame is the actual image frame
    ret, frame = cap.read()

    # If the frame was not read successfully, break the loop
    if not ret:
        print("Error: Could not read frame.")
        break

    # --- Process the frame here ---
    # For example, you can display the frame:
    cv2.imshow("Frame", frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
