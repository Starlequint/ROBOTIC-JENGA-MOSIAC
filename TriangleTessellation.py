from itertools import combinations
import numpy as np
import matplotlib.pyplot as plt


def triangle_aspect(p1, p2, p3):
    #Returns sorted side-length ratios. Equilateral = [1, 1, 1].
    sides = sorted([
        np.linalg.norm(p2[:2] - p1[:2]),
        np.linalg.norm(p3[:2] - p2[:2]),
        np.linalg.norm(p1[:2] - p3[:2])
    ])
    return np.array(sides) / sides[-1]

def is_plausible_triangle(p1, p2, p3, tol=0.15):
    #TODO, determine acceptable Tol
    #Equilateral triangles have ratio between all sides equal ~ [1,1,1].
    ratios = triangle_aspect(p1, p2, p3)
    return np.allclose(ratios, [1.0, 1.0, 1.0], atol=tol)

def is_valid_size(p1, p2, p3, expected_size=182, tol=0.1):
    #Checks if the average side length of the triangle is close to the expected size.
    sides = [
        np.linalg.norm(p2[:2] - p1[:2]),
        np.linalg.norm(p3[:2] - p2[:2]),
        np.linalg.norm(p1[:2] - p3[:2])
    ]
    avg_side = np.mean(sides)
    print(f"Average side length: {avg_side:.2f}, Expected size: {expected_size}")
    return expected_size * (1 - tol) <= avg_side <= expected_size * (1 + tol)

def is_valid_rotation(p1, p2, tol=10):
    #Checks if the rotation difference between two points is approximately 60 degrees (or 300 degrees)
    #TODO make work negative angles
    rot_diff = (p2[2] - p1[2])

    print(p2[2], p1[2], rot_diff)
    return (60 - tol <= rot_diff <= 60 + tol) or (-60 - tol <= rot_diff <= -60 + tol)

def plot_triangles(detected, triangles):
    if not triangles:
        print("No triangles to plot.")
        return

    segment_length = 182
    half_segment = segment_length / 2.0
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(detected[:, 0], detected[:, 1], 'x', color='blue', label='Detected Points')
    for idx, triangle in enumerate(triangles, start=1):
        # Close the triangle by repeating the first vertex for plotting.
        xz = triangle[:, :2]
        closed_xz = np.vstack([xz, xz[0]])
        
        for point in triangle:
            x_center, y_center, orientation_deg = point
            theta = np.deg2rad(orientation_deg)
            dx = half_segment * np.cos(theta)
            dy = half_segment * np.sin(theta)

            # Segment centered at (x, y) with orientation given by angle.
            x1, y1 = x_center - dx, y_center - dy
            x2, y2 = x_center + dx, y_center + dy
            ax.plot([x1, x2], [y1, y2], color="tab:red", linewidth=2)
            ax.scatter([x_center], [y_center], color="green", s=12)

    ax.set_title("Generated Triangle Tessellation")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    if len(triangles) <= 10:
        ax.legend(loc="best")
    plt.tight_layout()
    plt.show()

def patternRecognition(detected_points):
    detected = np.array(detected_points)  # Nx3 array, (x,y,rotation)

    candidates = []

    for i, j, k in combinations(range(len(detected)), 3):
        p1, p2, p3 = detected[i], detected[j], detected[k]
        if is_plausible_triangle(p1, p2, p3): 
            #Candidates now contains all triplets that form equilateral-ish triangles
            print(f"Found plausible triangle with points {i}, {j}, {k}"+
                  f" and aspect ratios {triangle_aspect(p1, p2, p3)}")
            if is_valid_size(p1, p2, p3): 
                #Sorts out triangles that are too small or too large to be the target
                print(f"Triangle with points {i}, {j}, {k} has acceptable size.")
                if sum([is_valid_rotation(p1, p2), is_valid_rotation(p1, p3), 
                        is_valid_rotation(p2, p3)]) == 2: 
                    #checks if blocs have correct roations to be equilateral.
                    candidates.append((i, j, k))

    #Generate tesselation from matched triangle
    final_triangles = []

    if candidates == []:
        print("No valid triangle candidates found.")
    else:
        base_triangle = detected[list(candidates[0])].copy() 
        final_triangles.append(base_triangle)
        print(f"Found {len(candidates)} valid triangle candidates.")
        side_len = 182 *2
            
        angle_1= base_triangle[0][2] 
            
        offset1 = np.array([np.cos(np.deg2rad(angle_1)) * side_len, 
                            np.sin(np.deg2rad(angle_1)) * side_len, 0])
        offset2 = np.array([np.cos(np.deg2rad(angle_1 + 60)) * side_len, 
                            np.sin(np.deg2rad(angle_1 + 60)) * side_len, 0])
        offset3 = np.array([np.cos(np.deg2rad(angle_1 - 60)) * side_len, 
                            np.sin(np.deg2rad(angle_1 - 60)) * side_len, 0])

        final_triangles.append(base_triangle + offset1)
        final_triangles.append(base_triangle - offset1)
        final_triangles.append(base_triangle + offset2)
        final_triangles.append(base_triangle - offset2)     
        final_triangles.append(base_triangle + offset3)
        final_triangles.append(base_triangle - offset3)

        #TODO add x and y limits, Maybe more appropriate in main seeing 
        # as we need to be in world coordinates
    \
    plot_triangles(detected, final_triangles) 
    return final_triangles # [(cx, cy, θ), ...]

# detected_points = [ #TEST IM 4
#     (573.7, 321.5, -73.80),   
#     (703.1, 432.3, -15.42),
#     (206.0, 579.8, -7.94),
#     (160.4, 225.2, -79.13),
#     (756.6, 258.6, 49.84),
#     (385.3, 144.4, 86.76),
#     (1040.2, 278.1, -76.61),
#     (1207.8, 335.6, -59.04)
# ]

# T = patternRecognition(detected_points)

# print("detected :", len(detected_points))
# print("recognized :", len(T), len(T[0]))
# print(T)
