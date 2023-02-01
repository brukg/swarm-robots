import numpy as np
import cv2
ix,iy,sx,sy = -1,-1,-1,-1
shape = []
polygone = None
r = None
# mouse callback function
def draw_lines(event, x, y, flags, param):
    global ix,iy,sx,sy,shape, img, polygone
    # if the left mouse button was clicked, record the starting
    
    if event == cv2.EVENT_LBUTTONDOWN:
        print("left button down")
        # draw circle of 2px
        cv2.circle(img, (x, y), 3, (0, 0, 127), -1)

        if ix != -1: # if ix and iy are not first points, then draw a line
            cv2.line(img, (ix, iy), (x, y), (0, 0, 127), 2, cv2.LINE_AA)
            shape.append((x, y))
        else: # if ix and iy are first points, store as starting points
            sx, sy = x, y
            shape.append((x, y))

        ix,iy = x, y
        
    elif event == cv2.EVENT_LBUTTONDBLCLK:
        ix, iy = -1, -1 # reset ix and iy
        if flags == 33: # if alt key is pressed, create line between start and end points to create polygon
            cv2.line(img, (x, y), (sx, sy), (0, 0, 127), 2, cv2.LINE_AA)
            print("double click")
            shape.append((x, y))
    elif event == cv2.EVENT_MBUTTONDOWN:
        # selct random two from shape and add mid points to the shape is size is less than 32
        while len(shape) < 10:
            rand = np.random.choice(len(shape), replace=False)
            shape.append(((shape[rand][0]+shape[rand-1][0])/2, (shape[rand][1]+shape[rand-1][1])/2))
        while len(shape) > 10:
            rand = np.random.choice(len(shape), replace=False)
            shape.pop(rand)
        shape =  np.array(shape).reshape(-1, 2)
        polygone = shape/50 # scale down to 1/40th of the image size to the map size of 8m x 8m
        # convert from image coordinates to map coordinates
        center_x, center_y = 4, 4
        polygone[:, 0] = polygone[:, 0] - center_x
        polygone[:, 1] = center_y - polygone[:, 1]

        # print("Shape: ", shape)
        # clear the lines
        img = np.zeros((400,400,3), np.uint8)
        ix, iy = -1, -1 # reset ix and iy
        shape = []
        # return s

def get_shape():
    global polygone, r
    if type(polygone) is np.ndarray:
      r = np.copy(polygone)
      polygone = None
    else:
      r = None
    # print("Shape: ", r)
    return r
# read image from path and add callback
# img = cv2.resize(cv2.imread("themefoxx (214).jpg"), dsize=(800, 600))

img = np.zeros((400,400,3), np.uint8)
cv2.namedWindow('input shape') 
cv2.setMouseCallback('input shape',draw_lines)

def main():
  while(1):
    cv2.imshow('input shape',img)
    if cv2.waitKey(20) & 0xFF == 27:
        break

  cv2.destroyAllWindows()