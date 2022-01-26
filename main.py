from PIL import Image
import numpy as np
import heapq
from itertools import combinations

# Used to identify the points of interest
location_color = (255,0,0,255)
location_colors = set([location_color])
wall_color = (0,0,0,255)
road_color = (251, 242, 54, 255)
air_color = (255, 255, 255, 255)
default_colors = set([location_color, road_color, air_color])

default_movement = [(0,1), (1,0), (-1, 0), (0, -1)]

def get_path(offers, start_posns, goal_posns):
    end_pos = [o for o in offers if o in goal_posns]
    # No path exists
    if len(end_pos) == 0:
        return None
    current_pos = end_pos[0]
    path = []
    while current_pos not in start_posns:
        prev_pos = offers[current_pos]
        path.insert(0, prev_pos)
        current_pos = prev_pos
    return path

def euc(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def image_search(image, start_posns, movement_rules, ok_colors, goal_posns):
    offers = {}
    finished = set()
    h = []
    def get_priority(p):
        # Lower distance is good
        if goal_posns is not None:
            eucs = [euc(p, goal_p) for goal_p in goal_posns]
            return min(eucs)
        else:
            return 0
    for p in start_posns:
        heapq.heappush(h, (get_priority(p), p))
    while len(h) != 0:
        priority, pos = heapq.heappop(h)
        if goal_posns is not None and pos in goal_posns:
            break
        next_positions = []
        for p in movement_rules:
            p_new = tuple(np.array(pos) + np.array(p))
            # Nothing new if it goes past the edge
            if p_new[0] < 0 or p_new[0] >= image.shape[0]:
                continue
            if p_new[1] < 0 or p_new[1] >= image.shape[1]:
                continue
            color = tuple(image[p_new])
            #TODO: different costs for different colors
            if color in ok_colors:
                next_positions.append(p_new)
        for next_pos in next_positions:
            if next_pos not in finished:
                offers[next_pos] = pos
                finished.add(next_pos)
                heapq.heappush(h, (get_priority(next_pos), next_pos))
    if goal_posns is not None:
        path = get_path(offers, start_posns, goal_posns)
    else:
        path = None
    return path, offers, finished

def find_locations(image_array, colors):
    locations = set()
    for x in range(image_array.shape[0]):
        for y in range(image_array.shape[1]):
            xyc = tuple(image_array[x,y,:])
            if xyc in colors:
                locations.add((x,y))
    return locations

def get_components(image_array, locations, movement_rules, colors):
    # Set of position tuples
    components = []
    while len(locations) > 0:
        root = next(iter(locations))
        _, _, f = image_search(image_array, [root], movement_rules, colors, None)
        components.append(f)
        locations = locations - f
    return components

def path_length(p):
    if p is None:
        return float("Inf")
    else:
        return len(p)
            
if __name__ == "__main__":
    image = Image.open("minimetroid_concept.png")
    image_array = np.array(image)
    locs = find_locations(image_array, location_colors)
    print(len(locs))
    components = get_components(image_array, locs, default_movement, location_colors)
    for c1, c2 in combinations(components, 2):
        print(next(iter(c1)))
        print(next(iter(c2)))
        p, _, _ = image_search(image_array, c1, default_movement, default_colors, c2)
        print(path_length(p))


