from maix import camera, display, image
import os
from CocoPi import multiFuncGpio

# Initialize line sensors
_PWN_gpio_1 = multiFuncGpio(0, 4)  # GPIO0 = left sensor = S1 = white cable
_PWN_gpio_2 = multiFuncGpio(1, 4)  # GPIO1 = right sensor = S2 = yellow cable

# Camera configuration
cameraSize = os.path.exists("/etc/cameraSize.cfg")
if cameraSize:
    camera.camera.config(size=(320, 240))
else:
    camera.camera.config(size=(240, 320))

# Load font for display
image.load_freetype("/root/preset/fonts/simhei.ttf")

def show_sensor_data():
    while True:
        # Capture camera image
        img = camera.capture()
        
        # Get sensor readings
        left_val = _PWN_gpio_1.analogRead()
        right_val = _PWN_gpio_2.analogRead()
        
        # Create display canvas
        canvas = image.new(size=(320, 240))
        
        # Draw sensor data
        canvas.draw_string(10, 50, f"Left: {left_val}", scale=2, color=(255, 255, 255), thickness=1)
        canvas.draw_string(10, 100, f"Right: {right_val}", scale=2, color=(255, 255, 255), thickness=1)
        
        # Add visual indicators
        canvas.draw_rectangle(50, 150, 100, 200, color=(255, 0, 0) if left_val < 1500 else (0, 255, 0), thickness=-1)
        canvas.draw_rectangle(200, 150, 250, 200, color=(255, 0, 0) if right_val < 1500 else (0, 255, 0), thickness=-1)
        
        # Show on display
        display.show(canvas)

if __name__ == "__main__":
    show_sensor_data()