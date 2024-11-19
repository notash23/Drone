import cv2
import os

def resize_image():
    # dimension = (750, 560)

    for index, image in enumerate(os.listdir('robot')):
        original_image = cv2.imread(f'robot/{image}')
        os.remove(f'robot/{image}')

        # Resize the image
        # resized_image = cv2.resize(original_image, dimension)

        # Save or display the image
        cv2.imwrite(f'robot/{index}.jpg', original_image)


resize_image()
