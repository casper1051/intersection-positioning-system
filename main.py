#!/usr/bin/python3
import os, sys
from collections import deque
sys.path.append("/usr/lib")
import _kipr as k

from tkinter import Tk, Canvas, NW
from PIL import Image, ImageTk
import math

robot_x = 1385
robot_y = 426

left_motor = 0
right_motor = 1

follow_sensor = 0
intersection_sensor = 1

line_thresh = 3700


def turn(angle):
    angle = angle * -1
    
    #upgrate turning in the cuture
    speed = 700
    ms_per_degree = 1700 / 90

    direction = 1 if angle > 0 else -1
    duration = int(abs(angle) * ms_per_degree)

    # Set wheel directions
    k.mav(left_motor, direction * speed)
    k.mav(right_motor, -direction * speed)
    k.msleep(duration)

    # Stop motors
    k.mav(left_motor, 0)
    k.mav(right_motor, 0)
def go_forward(feet):
    ms_per_foot = 3000  # from testing reference
    speed = 700

    duration = int(feet * ms_per_foot)
    
    print(f"Moving forward {feet} ft for {duration} ms at speed {speed}")
    k.mav(left_motor, speed)
    k.mav(right_motor, speed)
    k.msleep(duration)
    k.mav(left_motor, 0)
    k.mav(right_motor, 0)
def follow_until_intersection():
    while(k.analog(intersection_sensor) < line_thresh):
          if k.analog(follow_sensor) > line_thresh:
              k.mav(left_motor, 900)
              k.mav(right_motor, 700)
          if k.analog(follow_sensor) < line_thresh:
              k.mav(left_motor, 700)
              k.mav(right_motor, 900)
    go_forward(0.2)

def follow_for_distance(feet):
    ticks_per_foot = 1676
    target_ticks = int(feet * ticks_per_foot)
    
    k.clear_motor_position_counter(left_motor)
    k.clear_motor_position_counter(right_motor)

    print(f"Following line for {feet:.2f} feet ({target_ticks} ticks)")
    
    while k.gmpc(left_motor) < target_ticks:
        val = k.analog(follow_sensor)
        if val > line_thresh:
            k.mav(left_motor, 900)
            k.mav(right_motor, 700)
        elif val < line_thresh:
            k.mav(left_motor, 700)
            k.mav(right_motor, 900)
    
    k.mav(left_motor, 0)
    k.mav(right_motor, 0)

def follow_until_end():
    k.clear_motor_position_counter(left_motor)
    k.clear_motor_position_counter(right_motor)

    print(f"ImplementationRequiredError: >>>$ln89")
    
    while True:
        val = k.analog(follow_sensor)
        if val > line_thresh:
            k.mav(left_motor, 900)
            k.mav(right_motor, 700)
        elif val < line_thresh:
            k.mav(left_motor, 700)
            k.mav(right_motor, 900)
    
    k.mav(left_motor, 0)
    k.mav(right_motor, 0)

PX_TO_FEET = 1 / 142

def is_black(pixel):
    return all(c < 100 for c in pixel)

def snap_to_black_pixel(img, point, max_radius=10):
    x0, y0 = point
    width, height = img.size
    nearest = None
    nearest_dist = max_radius + 1
    for dx in range(-max_radius, max_radius + 1):
        for dy in range(-max_radius, max_radius + 1):
            nx, ny = x0 + dx, y0 + dy
            if 0 <= nx < width and 0 <= ny < height:
                if is_black(img.getpixel((nx, ny))):
                    dist = abs(dx) + abs(dy)
                    if dist < nearest_dist:
                        nearest_dist = dist
                        nearest = (nx, ny)
    return nearest if nearest else point

def check_line_segment(img, start, end, axis='horizontal', thickness=15):
    width, height = img.size
    
    if axis == 'horizontal':
        y = start[1]
        for x in range(start[0], end[0] + 1):
            for dy in range(-thickness, thickness + 1):
                ny = y + dy
                if 0 <= x < width and 0 <= ny < height:
                    if is_black(img.getpixel((x, ny))):
                        return True
        return False
    elif axis == 'vertical':
        x = start[0]
        for y in range(start[1], end[1] + 1):
            for dx in range(-thickness, thickness + 1):
                nx = x + dx
                if 0 <= nx < width and 0 <= y < height:
                    if is_black(img.getpixel((nx, y))):
                        return True
        return False

def is_intersection(img, point, segment_len=40, threshold=0.8):
    x, y = point
    width, height = img.size

    if not is_black(img.getpixel((x, y))):
        return False

    horiz = [
        (x + dx, y) for dx in range(-segment_len // 2, segment_len // 2)
        if 0 <= x + dx < width
    ]
    vert = [
        (x, y + dy) for dy in range(-segment_len // 2, segment_len // 2)
        if 0 <= y + dy < height
    ]

    horiz_black = sum(1 for px in horiz if is_black(img.getpixel(px))) / len(horiz)
    vert_black = sum(1 for px in vert if is_black(img.getpixel(px))) / len(vert)

    return horiz_black >= threshold and vert_black >= threshold

def bfs_find_nearest_black(img, start, max_radius=100):
    width, height = img.size
    visited = set()
    q = deque()
    q.append(start)

    while q:
        x, y = q.popleft()
        if (x, y) in visited:
            continue
        visited.add((x, y))

        if 0 <= x < width and 0 <= y < height:
            if is_black(img.getpixel((x, y))):
                return (x, y)
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = x + dx, y + dy
                if (0 <= nx < width and 0 <= ny < height and
                    abs(nx - start[0]) + abs(ny - start[1]) <= max_radius):
                    q.append((nx, ny))
    return None

def bfs_path(img, start, goal):
    width, height = img.size
    visited = set()
    came_from = {}
    q = deque()
    q.append(start)
    visited.add(start)

    while q:
        x, y = q.popleft()
        if (x, y) == goal:
            break
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < width and 0 <= ny < height
                and is_black(img.getpixel((nx, ny)))
                and (nx, ny) not in visited):
                visited.add((nx, ny))
                came_from[(nx, ny)] = (x, y)
                q.append((nx, ny))

    if goal not in came_from:
        print("Goal was not reached.")
        return []

    path = []
    curr = goal
    while curr != start:
        path.append(curr)
        curr = came_from[curr]
    path.append(start)
    path.reverse()
    return path

def angle_between(p1, p2, p3):
    """Calculate angle between vector p1->p2 and p2->p3 in degrees."""
    def vec(a, b):
        return (b[0] - a[0], b[1] - a[1])
    v1 = vec(p2, p1)
    v2 = vec(p2, p3)
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    det = v1[0]*v2[1] - v1[1]*v2[0]
    angle = math.atan2(det, dot) * 180 / math.pi
    return angle  # Positive = left turn, Negative = right turn

def path_to_instructions(img, path):
    if len(path) < 2:
        return []

    instructions = []
    segment_start = path[0]

    MIN_DIST_FEET = 0.05 #pixelwise detection on thick lines = 30px x 30px = 900
    MIN_ANGLE_DEG = 10      # same here
    INTERSECTION_MERGE_RADIUS = 160  #intersection aera = 900 --> 30x30, ignore in 160 radius to prevent spamming intersections

    seen_intersections = []

    i = 1
    while i < len(path):
        curr = path[i]

        # detect new intersections, filter out near-duplicates
        if is_black(img.getpixel(curr)) and is_intersection(img, curr):
            if not any(math.dist(curr, pt) < INTERSECTION_MERGE_RADIUS for pt in seen_intersections):
                seen_intersections.append(curr)
                instructions.append("follow_until_intersection()")
                segment_start = curr
                i += 1
                continue

        # turning
        if i + 1 < len(path):
            next_pt = path[i + 1]

            prev_vec = (curr[0] - path[i - 1][0], curr[1] - path[i - 1][1])
            next_vec = (next_pt[0] - curr[0], next_pt[1] - curr[1])

            prev_len = math.hypot(*prev_vec)
            next_len = math.hypot(*next_vec)

            if prev_len == 0 or next_len == 0:
                i += 1
                continue

            prev_dir = (prev_vec[0] / prev_len, prev_vec[1] / prev_len)
            next_dir = (next_vec[0] / next_len, next_vec[1] / next_len)

            dot = prev_dir[0] * next_dir[0] + prev_dir[1] * next_dir[1]

            if dot < 0.95:
                dist_px = math.dist(segment_start, curr)
                dist_ft = dist_px * PX_TO_FEET

                if dist_ft > MIN_DIST_FEET:
                    instructions.append(f"follow_for_distance({dist_ft:.2f})")

                angle = angle_between(path[i - 1], curr, next_pt)

                if abs(angle) > MIN_ANGLE_DEG:
                    instructions.append(f"turn({angle:.1f})")

                segment_start = curr
                i += 1
                continue

        i += 1

    #last part
    dist_px = math.dist(segment_start, path[-1])
    dist_ft = dist_px * PX_TO_FEET

    if dist_ft > MIN_DIST_FEET:
        instructions.append("follow_until_end()")

    if not is_black(img.getpixel(path[-1])):
        instructions.append("go_forward(0.5)  # move off line to goal")

    return instructions

def show_map():
    root = Tk()
    root.title("Map Viewer")

    map_path = os.path.expanduser("~/map.png")
    try:
        pil_img_original = Image.open(map_path).convert("RGB")
    except FileNotFoundError:
        print("Could not find ~/map.png")
        return

    canvas_width, canvas_height = 800, 600
    pil_img = pil_img_original.copy()
    pil_img.thumbnail((canvas_width, canvas_height), Image.ANTIALIAS)
    width, height = pil_img.size

    scale_x = pil_img_original.width / width
    scale_y = pil_img_original.height / height

    tk_img = ImageTk.PhotoImage(pil_img)
    canvas = Canvas(root, width=canvas_width, height=canvas_height)
    canvas.pack()
    offset_x = (canvas_width - width) // 2
    canvas.create_image(offset_x, 0, anchor=NW, image=tk_img)

    #bot
    rx = int(robot_x / scale_x) + offset_x
    ry = int(robot_y / scale_y)
    canvas.create_oval(rx - 5, ry - 5, rx + 5, ry + 5, fill='green', outline='black')

    def draw_path(path, color="green"):
        for (x, y) in path:
            cx = int(x / scale_x) + offset_x
            cy = int(y / scale_y)
            r = 2
            pixel = pil_img_original.getpixel((x, y))
            if is_intersection(pil_img_original, (x, y)):
                r = 5
                canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill='red', outline='black')
            else:
                r = 2
                canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=color, outline=color)

    def on_click(event):
        gx = int((event.x - offset_x) * scale_x)
        gy = int(event.y * scale_y)

        if not (0 <= gx < pil_img_original.width and 0 <= gy < pil_img_original.height):
            print("Click out of bounds")
            return

        robot_pos = (robot_x, robot_y)
        goal_pos = snap_to_black_pixel(pil_img_original, (gx, gy))
        print(f"Snapped goal to {goal_pos}")

        print(f"Goal: {goal_pos}")
        
        print("Performing BFS Pathfinding to locate nearest black line...")

        nearest = bfs_find_nearest_black(pil_img_original, robot_pos)
        
        print("Complete")
        
        if not nearest:
            print("No nearby black pixel found")
            return

        print("Performing BFS Pathfinding to locate path to goal...")
        path_to_goal = bfs_path(pil_img_original, nearest, goal_pos)
        if not path_to_goal:
            print("Couldn't pathfind to goal along black line. Drawing to black pixel only.")
            path_to_goal = []
            
        print("Complete")

        full_path = []

        #locate closest line
        full_path += line(robot_pos, nearest)

        #pathing
        full_path += path_to_goal

        instructions = path_to_instructions(pil_img_original, full_path)
        print("Generated instructions:")
        for instr in instructions:
            print(instr)

        draw_path(full_path)
        
        command_entered = input("Eval() instructions? y/n")
        
        if command_entered == "y":
        	instructions_code = "\n".join(instructions)
        	exec(instructions_code)

    def line(p0, p1):
        x0, y0 = p0
        x1, y1 = p1
        points = []

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return points

    canvas.bind("<Button-1>", on_click)
    root.mainloop()

if __name__ == "__main__":
    show_map()
