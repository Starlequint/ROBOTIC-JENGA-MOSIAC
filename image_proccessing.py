from cv2 import (imread, arrowedLine, circle, cvtColor, COLOR_BGR2RGB, COLOR_BGR2HSV, 
    createCLAHE, GaussianBlur, Sobel, CV_64F, threshold, THRESH_BINARY, 
    getStructuringElement, MORPH_RECT, dilate, IMREAD_GRAYSCALE, Canny, RETR_LIST, 
    findContours, CHAIN_APPROX_SIMPLE, imwrite, minAreaRect, contourArea, convexHull)
import numpy as np
from matplotlib.pyplot import subplots, show, tight_layout
from math import radians, cos, sin

def draw_detections(image_path, results, arrow_length=40, dot_radius=5,dot_color=(0, 0, 255), arrow_color=(0, 0, 255), thickness=2):
    """
    Overlay detection results on the original image.

    Parameters
    ----------
    image_path   : str   – path to the original image (raw or color)
    results      : list  – list of (cx, cy, angle_deg) tuples angle is the orientation of the longest side vs x-axis
    arrow_length : int   – half-length of the direction arrow in pixels
    dot_radius   : int   – radius of the center dot
    dot_color    : tuple – BGR color for the dot
    arrow_color  : tuple – BGR color for the arrow
    thickness    : int   – line thickness for the arrow

    Returns
    -------
    vis : np.ndarray – annotated image (BGR)
    """
    img = imread(image_path)
    vis = img.copy()

    for (cx, cy, angle_deg) in results:
        cx, cy = int(round(cx)), int(round(cy))

        # Convert angle to radians
        # angle_deg is the angle of the longest side w.r.t. the x-axis
        rad = radians(angle_deg)

        # Compute arrow tip and tail (centered on the plank center)
        dx = int(arrow_length * cos(rad))
        dy = int(arrow_length * sin(rad))

        pt_tail = (cx, cy)
        pt_tip  = (cx + dx, cy + dy)

        # Draw arrow
        arrowedLine(vis, pt_tail, pt_tip,
                        color=arrow_color,
                        thickness=thickness,
                        tipLength=0.3)

        # Draw center dot
        circle(vis, (cx, cy), dot_radius, dot_color, -1)

    return vis

def draw_points(image_path, points, dot_radius=5,dot_color=(0, 0, 255)):
    img = imread(image_path)
    vis = img.copy()

    for (cx, cy, angle_deg) in points:
        cx, cy = int(round(cx)), int(round(cy))
        pt_ = (cx, cy)
        # Draw center dot
        circle(vis, (cx, cy), dot_radius, dot_color, -1)
    return vis


def show_overlay(image_path, results, arrow_length=40, figsize=(14, 8), **kwargs):
    """
    Display the annotated image inline in a Jupyter notebook.

    Parameters
    ----------
    image_path   : str   – path to the original image
    results      : list  – list of (cx, cy, angle_deg) tuples
    arrow_length : int   – half-length of the direction arrow in pixels
    figsize      : tuple – matplotlib figure size
    **kwargs     : extra keyword args forwarded to draw_detections
    """
    #show point + arrow
    vis = draw_detections(image_path, results, arrow_length=arrow_length, **kwargs)
    
    #show points only
    #vis= draw_points(image_path, results, **kwargs)
    
    # OpenCV loads BGR, matplotlib expects RGB
    vis_rgb = cvtColor(vis, COLOR_BGR2RGB)

    fig, ax = subplots(figsize=figsize)
    ax.imshow(vis_rgb)
    ax.axis('off')
    ax.set_title(f"{len(results)} plank(s) detected", fontsize=13)
    tight_layout()
    show()

    return vis

#images to use
#color img:
my_color_img='Test_images12_raw_img_work_with_them\\normalized_color11.png'
#grey img:
my_grey_img='Test_images12_raw_img_work_with_them\\raw_image11.png'

#Extract hue as depth proxy
color_img = imread(my_color_img)
hsv = cvtColor(color_img, COLOR_BGR2HSV)
hue= hsv[:, :, 0]   #depth map, 0–179 in OpenCV

#CLAHE on the hue channel (for local contrast)
clahe     = createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
hue_eq    = clahe.apply(hue)

#Blur to reduce noise
blurred   = GaussianBlur(hue_eq, (3, 3), 0)

#Canny on the hue channel — thresholds likely need to be different
#from grayscale since hue is 0-179, not 0-255
#edges = cv2.Canny(blurred, threshold1=10, threshold2=40)

#or, instead of Canny, using Sobel:

sobelx = Sobel(blurred, CV_64F, 1, 0, ksize=3)
sobely = Sobel(blurred, CV_64F, 0, 1, ksize=3)
magnitude = np.sqrt(sobelx**2 + sobely**2)

# Normalize and threshold to get binary edge image
magnitude = np.uint8(255 * magnitude / magnitude.max())
_, edges= threshold(magnitude, 30, 255, THRESH_BINARY)  # tune 30

#end of sobel

#Close gaps --> no visible improvement
kernel       = getStructuringElement(MORPH_RECT, (7, 7))
edges_closed = dilate(edges, kernel, iterations=1)
edges_closed=edges

img_grey = imread(my_grey_img, IMREAD_GRAYSCALE)

#increase contrast (to avoid prblms in darker regions)
#clahe = cv2.createCLAHE(clipLimit=2.3, tileGridSize=(8, 8))  # tune clipLimit: 1.0–4.0
clahe = createCLAHE(clipLimit=2, tileGridSize=(4, 4))  # tune clipLimit: 1.0–4.0
img_eq = clahe.apply(img_grey)
#edges_grey  = cv2.Canny(img1, threshold1=30, threshold2=100)

#gaussian filter
blurred = GaussianBlur(img_eq, (11, 11), 0) #(5,5)

#edge detection, works better with manual set thresholds
#median  = np.median(blurred)
#lower   = int(max(0,   0.66 * median))
#upper   = int(min(255, 1.33 * median))
#edges   = cv2.Canny(blurred, lower, upper)
#print(lower, upper)
edges_grey = Canny(blurred, threshold1=30, threshold2=100)#30, 100

#close gaps in the edges to try to get rectangular shape --> not useful
kernel = getStructuringElement(MORPH_RECT, (3, 3)) #tune size: try 3,5,7
edges_grey = dilate(edges_grey, kernel, iterations=1)     # try iterations=2 if gaps persist

contours, _  = findContours(edges_closed, RETR_LIST, CHAIN_APPROX_SIMPLE)

edges_closed=edges_grey+edges_closed

fig, ax = subplots(1, 3, figsize=(15, 7))

# Original Image
ax[0].imshow(cvtColor(color_img, COLOR_BGR2RGB))
ax[0].set_title('Original')
ax[0].axis('off')

# Blurred Image
ax[1].imshow(blurred, cmap='gray')
ax[1].set_title('Gaussian Blur')
ax[1].axis('off')

ax[2].imshow(edges, cmap='gray')
ax[2].set_title('Edges')
ax[2].axis('off')

show()

#save 'edges'
imwrite('detected_edges_img.png', edges_closed)

results = []
print('Raw detected features fulfilling area condition:')

for cnt in contours:
    area = contourArea(cnt)
#    if area < 3000:  # tune this
    if area < 8000:   # only skip tiny noise <=> if area < XXX, don't consider it, else consider it
        continue
        
    rect = minAreaRect(cnt)
    (cx, cy), (w, h), angle = rect
    if w < h:
        w, h = h, w
        angle += 90
        
    aspect = w / h if h > 0 else 0
    hull = convexHull(cnt)
    solidity = area / contourArea(hull)
    
    print(f"cx={cx:.0f} cy={cy:.0f} | area={area:.0f} | "
          f"aspect={aspect:.2f} | solidity={solidity:.2f} | angle={angle:.1f}°")
    
    #if 1.5 < aspect < 7.0 and 0.65 < solidity < 0.95:  # tune thresholds
    if 2.0 < aspect < 5.0:
        results.append((cx, cy, angle))

#remove duplicates: points that are very close to each other
to_remove=[]
for i in range(0, len(results)):
    for j in range(i+1, len(results)):
        if (results[i][0]-results[j][0]) < 10 and (results[i][1]-results[j][1]) < 10:
            to_remove.append(results[j])
for elem in to_remove:
    if elem in results:
        results.remove(elem)

show_overlay('detected_edges_img.png', results)
show_overlay(my_color_img, results)
show_overlay(my_grey_img, results)
