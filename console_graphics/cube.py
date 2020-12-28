import math
import time
import curses


REFRESH_RATE = 30
GRAYS = [' ', '░', '▒', '▓', '█']
PLAYER_DIST = 2.25
PLAYER_HEIGHT = 0.5
# The camera will rotate at this rate, rad/frame w/ locked FPS
PLAYER_ANGULAR_VELOCITY = 2 * math.pi / 60  / 2  # REFRESH_RATE
SCREEN_DISTANCE = 0.5  # From player
HFOV = 80 * math.pi / 180  # Roughly 1 deg / pixel if term is 80 columns wide
VFOV = 60 * math.pi / 180

LIGHT_DISTANCE = 5
LIGHT_BRIGHTNESS = 2
AMBIENT_LIGHT = 1
SATURATION_LEVEL = 5


def z_rotate(vector, angle_rad):
    return Vector(
        vector.x * math.cos(angle_rad) - vector.y * math.sin(angle_rad),
        vector.x * math.sin(angle_rad) + vector.y * math.cos(angle_rad),
        vector.z
    )


class Vector(object):
    __slots__ = 'x', 'y', 'z', '_length'
    def __init__(self, x, y, z):
        self.x, self.y, self.z = float(x), float(y), float(z)
        self._length = None
    def copy(self):
        return Vector(self.x, self.y, self.z)
    def __add__(self, other_vector):
        return Vector(self.x + other_vector.x, self.y + other_vector.y, self.z + other_vector.z)
    def __sub__(self, other_vector):
        return Vector(self.x - other_vector.x, self.y - other_vector.y, self.z - other_vector.z)
    def __mul__(self, scale_factor):
        return Vector(self.x * scale_factor, self.y * scale_factor, self.z * scale_factor)
    def __rmul__(self, scale_factor):
        return self.__mul__(scale_factor)
    def dot(self, other_vector):
        return (self.x * other_vector.x) + (self.y * other_vector.y) + (self.z * other_vector.z)
    def cross(self, other_vector):
        return Vector(
            self.y * other_vector.z - self.z * other_vector.y,
            -self.x * other_vector.z + self.z * other_vector.x,
            self.x * other_vector.y - self.y * other_vector.x
        )
    def __eq__(self, other_vector):
        return all([self.x == other_vector.x, self.y == other_vector.y, self.z == other_vector.z])
    def length(self):
        if self._length is None:
            self._length = math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
        return self._length
    def normalize(self):
        my_length = self.length()
        return Vector(self.x / my_length, self.y / my_length, self.z / my_length)
    def __repr__(self):
        return f"Vector(x={self.x}, y={self.y}, z={self.z})"
    

class Poly(object):
    __slots__ = 'vertices', 'surface_vector'
    def __init__(self, vertices):
        assert len(vertices) > 2
        self.vertices = vertices

        self.surface_vector = (
            self.vertices[1] - self.vertices[0]
        ).cross(
            self.vertices[2] - self.vertices[1]
        ).normalize()
    def contains_point(self, point):
        cross = (self.vertices[0] - self.vertices[-1]).cross(point - self.vertices[-1])
        for v1, v2 in zip(self.vertices[:-1], self.vertices[1:]):
            edge = v2 - v1
            to_point = point - v1
            if cross.dot(edge.cross(to_point)) < 0:
                return False
        return True
    def project_ray_onto_plane(self, base, unit_vec):
        param = self.surface_vector.dot(self.vertices[0] - base) / (self.surface_vector.dot(unit_vec))
        return base + param * unit_vec


def main(screen):

    # curses.curs_set(0)

    # Poly faces for a cube with sides length 1 and one corner on the origin
    # Defined with vertices in order so that the cross product of edges gives
    # a norm vector that points outside of the cube
    cube_polys = [
        Poly([     # X-Y plane face at Z = 0
            Vector(0, 0, 0),
            Vector(0, 1, 0),
            Vector(1, 1, 0),
            Vector(1, 0, 0),
        ]), Poly([  # X-Y plane face at Z = 1
            Vector(0, 0, 1),
            Vector(1, 0, 1),
            Vector(1, 1, 1),
            Vector(0, 1, 1),
        ]), Poly([  # Y-Z plane face at X = 0
            Vector(0, 0, 0),
            Vector(0, 0, 1),
            Vector(0, 1, 1),
            Vector(0, 1, 0),
        ]), Poly([  # Y-Z plane face at X = 1
            Vector(1, 0, 0),
            Vector(1, 1, 0),
            Vector(1, 1, 1),
            Vector(1, 0, 1),
        ]), Poly([  # X-Z plane face at Y = 0
            Vector(0, 0, 0),
            Vector(1, 0, 0),
            Vector(1, 0, 1),
            Vector(0, 0, 1),
        ]), Poly([  # X-Z plane face at Y = 1
            Vector(0, 1, 0),
            Vector(0, 1, 1),
            Vector(1, 1, 1),
            Vector(1, 1, 0),
        ])
    ]

    player_pos = Vector(math.sqrt(2) / 2 * PLAYER_DIST, math.sqrt(2) / 2 * PLAYER_DIST, PLAYER_HEIGHT)
    player_dir = (Vector(0, 0, PLAYER_HEIGHT) - player_pos).normalize()

    screen_height, screen_width = screen.getmaxyx()
    center_screen_row, center_screen_col = screen_height // 2, screen_width // 2

    v_pix_size = 2 * SCREEN_DISTANCE * math.tan(VFOV / 2) / screen_height
    h_pix_size = 2 * SCREEN_DISTANCE * math.tan(HFOV / 2) / screen_width

    light_location = Vector(0.5, 1, 0.5).normalize() * LIGHT_DISTANCE

    while True:
        
        t0 = time.time()

        view_angle = math.atan2(player_dir.y, player_dir.x)

        for row in range(screen_height-1):
            for col in range(screen_width-1):
                # In coordinate frame where y, z is defined by screen, y horiz L-R, z vert down to up
                # x is out of screen away from player
                # player is at (x,y,z) = (0, 0, 0)

                pix_vec = Vector(
                    SCREEN_DISTANCE,
                    (col - center_screen_col) * h_pix_size,
                    (center_screen_row - row) * v_pix_size  # Because curses y=0 is top of screen
                ).normalize()
                
                pix_vec = z_rotate(pix_vec, view_angle)

                polys_along_los = []

                for poly in cube_polys:
                    try:
                        plane_point = poly.project_ray_onto_plane(player_pos, pix_vec)
                    except ZeroDivisionError:
                        # We're edge-on to the plane, don't draw it
                        continue

                    if poly.contains_point(plane_point):
                        plane_dist = (plane_point - player_pos).length()
                        polys_along_los.append((plane_dist, plane_point, poly))

                pixel_selector = 0

                if polys_along_los:
                    dist_to_point, point_to_draw, poly = min(polys_along_los, key=lambda x: x[0])
                    poly_to_light = light_location - point_to_draw

                    dist_to_light = (poly_to_light).length()

                    if poly.surface_vector.dot(poly_to_light) > 0:
                        light_reflection = LIGHT_BRIGHTNESS #- 2 * dist_to_light
                    else:
                        light_reflection = 0

                    viz_brightness = (
                        light_reflection + AMBIENT_LIGHT # - 2 * dist_to_point
                    )
                    viz_brightness = min([viz_brightness, SATURATION_LEVEL - SATURATION_LEVEL / 1000])

                    pixel_selector = max([0, int((viz_brightness / SATURATION_LEVEL) * len(GRAYS))])

                screen.addstr(row, col, GRAYS[pixel_selector])
            
        player_pos = z_rotate(player_pos, PLAYER_ANGULAR_VELOCITY)
        player_dir = z_rotate(player_dir, PLAYER_ANGULAR_VELOCITY)

        t = time.time()
        fps = 1 / (t - t0)

        screen.addstr(0, 0, f'{fps:.0f}')

        screen.refresh()

        if fps > REFRESH_RATE:
            time.sleep((1 / REFRESH_RATE) - (t - t0))
                    

if __name__ == '__main__':

    curses.wrapper(main)



# Does ray intersect w/ Poly? At what point?

# To draw, 
#   Create a ray from camera, through each pixel on the display
#   Does ray intersect? If so, get the point
#   Measure distance to observer
#   Measure surface angle to light source and distance to light source
#   Shading is quantized gray scale... determine gray level
#   Fill pixel as appropriate
