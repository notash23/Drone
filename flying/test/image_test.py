import cv2
from djitellopy import Tello

d = Tello()
d.connect()
print(f"Battery Status: {d.get_battery()}")

# Start recording
d.streamon()
frame_read = d.get_frame_read()

d.takeoff()

while True:
    img = frame_read.frame
    print(img.shape)
    cv2.imshow('frame', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        d.streamoff()
        if d.is_flying:
            d.land()
        break
