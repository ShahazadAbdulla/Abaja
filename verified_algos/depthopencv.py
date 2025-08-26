import cv2
from ultralytics import YOLO

# --- FUNCTIONS FOR DISTANCE CALCULATION ---
def calculate_distance(focal_length, known_width, pixel_width):
    """
    Calculates the distance from the camera to a detected object using triangle similarity.
    :param focal_length: The focal length of the camera (must be pre-calibrated).
    :param known_width: The known real-world width of the object.
    :param pixel_width: The width of the object's bounding box in pixels.
    :return: The estimated distance in the same units as known_width.
    """
    if pixel_width == 0:
        return 0
    return (known_width * focal_length) / pixel_width

def main():
    # =================================================================================
    # TODO: IMPORTANT - UPDATE THESE VALUES FOR YOUR SPECIFIC SETUP
    # =================================================================================

    # --- STEP 1: Provide the absolute path to your YOLOv8 model ---
    # Example for Windows: "C:\\Users\\YourUser\\Documents\\models\\yolov8n.pt"
    # Example for Linux/Mac: "/home/user/models/yolov8n.pt"
    # This should be the model you trained or a standard YOLOv8 model.
    YOLO_MODEL_PATH = "/home/ashy/Downloads/yolov8n.pt"  # <--- CHANGE THIS

    # --- STEP 2: Calibrate and set the Focal Length of your camera ---
    # Run the calibration process described above to find this value.
    # This is a sample value; you MUST calculate your own for accurate results.
    FOCAL_LENGTH = 814  # A sample focal length in pixels (NEEDS CALIBRATION)

    # --- STEP 3: Define the known widths of objects your model can detect ---
    # Add the class names and their average real-world widths in the same unit (e.g., cm).
    # This is crucial for accuracy. You can add as many objects as you need.
    # These names MUST MATCH the class names in your YOLO model's training data.
    OBJECT_WIDTHS = {
        "person": 55,       # Average shoulder width in cm
        "car": 180,         # Average car width in cm
        "laptop": 35,       # Average laptop width in cm
        "cell phone": 7,    # Average phone width in cm
        "bottle": 7,        # Standard soda bottle width in cm
        "cup": 8,           # Standard cup width in cm
        "traffic light": 30,# Average traffic light width in cm
        "stop sign": 75     # Standard stop sign width in cm
        # Add other objects your model is trained to detect
    }
    
    # =================================================================================

    # Load the YOLOv8 model
    try:
        model = YOLO(YOLO_MODEL_PATH)
    except Exception as e:
        print(f"Error loading YOLO model from path: {YOLO_MODEL_PATH}")
        print(f"Error: {e}")
        return

    # Open the default camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Perform object detection on the frame
        results = model(frame)

        # Process detections
        for result in results:
            for box in result.boxes:
                # Get the class ID and confidence
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                
                # Get the class name from the model's names dictionary
                class_name = model.names[class_id]

                # Proceed only if the detected object has a known width
                if class_name in OBJECT_WIDTHS and confidence > 0.5:
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Calculate the pixel width of the bounding box
                    pixel_width = x2 - x1
                    print(f"Detected '{class_name}'. Pixel Width: {pixel_width}") # For calibration
                    
                    # Get the known real-world width from our dictionary
                    known_width = OBJECT_WIDTHS[class_name]
                    
                    # Calculate the distance
                    distance = calculate_distance(FOCAL_LENGTH, known_width, pixel_width)
                    
                    # --- Draw visuals on the frame ---
                    # Draw the bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Prepare the text to display (class name, confidence, and distance)
                    label = f"{class_name} ({confidence:.2f})"
                    distance_text = f"Distance: {distance:.2f} cm"
                    
                    # Put the text on the frame
                    cv2.putText(frame, label, (x1, y1 - 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.putText(frame, distance_text, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('YOLOv8 Real-Time Object Distance Estimation', frame)

        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()