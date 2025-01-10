# AI Virtual Mouse

A Python script that allows you to control your computer's mouse cursor using hand gestures captured through your webcam.

## Description
This project uses computer vision and hand tracking to create a virtual mouse control system. It detects hand movements and converts them into mouse actions, allowing you to control your cursor without touching your physical mouse.

## Requirements
- Python 3.x
- OpenCV
- Mediapipe
- AutoPy
- NumPy

## Installation
```bash
pip install opencv-python mediapipe autopy numpy
```

## Usage Instructions
1. Run the script:
    ```bash
    python virtual_mouse.py
    ```
2. Position your hand in front of the webcam
3. Use the following gestures:
    - Move your index finger to control cursor movement
    - Bring index and middle fingers together to perform a click
    - Hold index and middle fingers together to drag items
    - Hold both index and middle fingers up to hover the mouse cursor

## Note
- Ensure good lighting conditions for better hand detection
- Keep your hand within the camera frame
- Maintain appropriate distance from the camera for optimal tracking

## Contributing
Feel free to fork this project and submit pull requests for any improvements.

## License
This project is licensed under the MIT License.