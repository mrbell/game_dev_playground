'''
Draw a cube to the terminal and rotate the camera around the cube at a fixed rate. 
Trying to do it in pure Python and hit at least 20 FPS.
'''
import time
import curses
import numpy as np


REFRESH_RATE = 30
GRAYS = [' ', '░', '▒', '▓', '█']
PLAYER_DIST = 2.25
PLAYER_HEIGHT = 0.5
# The camera will rotate at this rate, rad/frame w/ locked FPS
PLAYER_ANGULAR_VELOCITY = 2 * np.pi / 20 / 2  # REFRESH_RATE
SCREEN_DISTANCE = 0.5  # From player
HFOV = 80 * np.pi / 180  # Roughly 1 deg / pixel if term is 80 columns wide
VFOV = 60 * np.pi / 180

LIGHT_DISTANCE = 5
LIGHT_BRIGHTNESS = 2
AMBIENT_LIGHT = 1
SATURATION_LEVEL = 5

EPS = 0.000_000_1


def z_rotate(vector, angle_rad):
    '''
    Rotate the given vector around the Z axis.
    '''
    cosa = np.cos(angle_rad)
    sina = np.sin(angle_rad)

    return Vector(*(np.array([[cosa, -sina, 0], [sina, cosa, 0], [0, 0, 1]]) @ vector.xyz))


class Vector(object):
    '''
    A basic 3D Vector class. Supports basic operations like addition, subtraction,
    scalar multiplication, dot product, cross product, normalization, and length
    calculation. Can be compared to other vectors for equality.
    '''
    __slots__ = 'xyz', '_length'
    def __init__(self, x, y, z):
        self.xyz = np.array([x, y, z])
        self._length = None
    def copy(self):
        return Vector(*self.xyz)
    def __add__(self, other_vector):
        return Vector(*(self.xyz + other_vector.xyz))
    def __sub__(self, other_vector):
        return Vector(*(self.xyz - other_vector.xyz))
    def __mul__(self, scale_factor):
        return Vector(*(self.xyz * scale_factor))
    def __rmul__(self, scale_factor):
        return self.__mul__(scale_factor)
    def dot(self, other_vector):
        return self.xyz @ other_vector.xyz
    def cross(self, other_vector):
        # return Vector(*np.cross(self.xyz, other_vector.xyz))
        return Vector(
            self.xyz[1] * other_vector.xyz[2] - self.xyz[2] * other_vector.xyz[1],
            -self.xyz[0] * other_vector.xyz[2] + self.xyz[2] * other_vector.xyz[0],
            self.xyz[0] * other_vector.xyz[1] - self.xyz[1] * other_vector.xyz[0]
        )
    def __eq__(self, other_vector):
        return self.xyz == other_vector.xyz
    def length(self):
        if self._length is None:
            self._length = np.linalg.norm(self.xyz)
        return self._length
    def normalize(self):
        my_length = self.length()
        return Vector(*(self.xyz / my_length))
    def __repr__(self):
        return f"Vector(x={self.xyz[0]}, y={self.xyz[1]}, z={self.xyz[2]})"


class Poly(object):
    '''
    A polygon defined by vertices. Edges are lines between subsequent pairs of 
    vertices, so ordering of vertices matters. The "front" of the poly, i.e. the
    direction of the surface normal vector, is inferred from the cross product of 
    the first two edges, so again, order of edges matters.
    '''
    
    __slots__ = 'vertices', 'surface_vector', 'edges'

    def __init__(self, vertices):
        assert len(vertices) > 2
        self.vertices = vertices

        edges = [self.vertices[0] - self.vertices[-1]]
        for v1, v2 in zip(self.vertices[:-1], self.vertices[1:]):
            edges.append(v2 - v1)
        self.edges = edges

        self.surface_vector = edges[1].cross(edges[2]).normalize()

    def __add__(self, other_vector):
        vertices = []
        for vertex in self.vertices:
            vertices.append(vertex + other_vector)
        return Poly(vertices)
    
    def __radd__(self, other_vector):
        return self.__add__(other_vector)

    def contains_point(self, point):
        '''
        Check whether a given point (as a Vector) is within the poly face.
        '''
        cross = self.edges[0].cross(point - self.vertices[-1])
        for edge, edge_base in zip(self.edges[1:], self.vertices[:-1]):
            to_point = point - edge_base
            # EPSILON here protects from points along an edge of the poly from getting registered
            # improperly
            if cross.dot(edge.cross(to_point)) <= 0:
                return False
        return True

    def project_ray_onto_plane(self, base, unit_vec):
        '''
        Calculate the point where the given ray (defined by a base vector and a unit vector along
        the ray) intersects with the plane of the poly face.
        '''
        denom = self.surface_vector.dot(unit_vec)
        if abs(denom) < EPS:
            return None
        param = self.surface_vector.dot(self.vertices[0] - base) / denom

        if param < 0:
            return None

        return base + param * unit_vec
    
    def ray_intersection(self, ray_base, ray_unit_vec):
        '''
        If the given ray intersects with the poly face, return the point of intersection,
        else return None.
        '''
        plane_intersection = self.project_ray_onto_plane(ray_base, ray_unit_vec)
        if plane_intersection is not None and self.contains_point(plane_intersection):
            return plane_intersection
        else:
            return None


def main(screen):

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

    # TESTING
    # cube_polys = [cube_poly + Vector(0, 0, 1000) for cube_poly in cube_polys]

    player_pos = Vector(np.sqrt(2) / 2 * PLAYER_DIST, np.sqrt(2) / 2 * PLAYER_DIST, PLAYER_HEIGHT)
    player_dir = (Vector(0, 0, PLAYER_HEIGHT) - player_pos).normalize()

    screen_height, screen_width = screen.getmaxyx()
    center_screen_row, center_screen_col = screen_height // 2, screen_width // 2

    v_pix_size = 2 * SCREEN_DISTANCE * np.tan(VFOV / 2) / screen_height
    h_pix_size = 2 * SCREEN_DISTANCE * np.tan(HFOV / 2) / screen_width

    light_location = Vector(0.5, LIGHT_DISTANCE, 0.5)
    n = 0
    while True: # n < 10:
        n += 1
        
        t0 = time.time()

        view_angle = np.arctan2(player_dir.xyz[1], player_dir.xyz[0])
        cosa = np.cos(view_angle)
        sina = np.sin(view_angle)
        R = np.array([[cosa, -sina, 0], [sina, cosa, 0], [0, 0, 1]])

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
                pix_vec = Vector(*(R @ pix_vec.xyz))

                polys_along_los = []

                for poly in cube_polys:
                    plane_point = poly.ray_intersection(player_pos, pix_vec)
                    if plane_point is not None:
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

        screen.addstr(0, 0, f'{fps:.4f}')

        screen.refresh()

        if fps > REFRESH_RATE:
            time.sleep((1 / REFRESH_RATE) - (t - t0))
                    

if __name__ == '__main__':

    curses.wrapper(main)
