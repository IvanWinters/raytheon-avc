import cv2
import os


cap = cv2.VideoCapture(0)

#num images
ctr = 0


current_directory = os.getcwd()
output_directory = os.path.join(current_directory, r'output')
os.makedirs(output_directory, exist_ok=True)

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    cv2.imshow('frame', frame)

    key = cv2.waitKey(1)

    #esc
    if key%256 == 27:
        break

    if key%256 == ord('s'):
        img = os.path.join(output_directory, "frame_{}.png".format(ctr))
        cv2.imwrite(img, frame)
        print(f"{ctr} written")
        ctr += 1


cap.release()
cv2.destroyAllWindows()

