'''
Draw a cube to the terminal and rotate the camera around the cube at a fixed rate. 
Trying to do it in pure Python and hit at least 20 FPS.
'''
import math
import time
import curses


REFRESH_RATE = 30
GRAYS = [' ', '░', '▒', '▓', '█']
PLAYER_DIST = 2.25
PLAYER_HEIGHT = 0.5
# The camera will rotate at this rate, rad/frame w/ locked FPS
PLAYER_ANGULAR_VELOCITY = 0  # 2 * math.pi / 20 / REFRESH_RATE
SCREEN_DISTANCE = 0.5  # From player
HFOV = 80 * math.pi / 180  # Roughly 1 deg / pixel if term is 80 columns wide
VFOV = 60 * math.pi / 180

LIGHT_DISTANCE = 5
LIGHT_BRIGHTNESS = 2
AMBIENT_LIGHT = 1
SATURATION_LEVEL = 5

EPS = 0.000_000_1


def z_rotate(vector, angle_rad):
    '''
    Rotate the given vector around the Z axis.
    '''
    cosa = math.cos(angle_rad)
    sina = math.sin(angle_rad)

    return Vector(
        vector.x * cosa - vector.y * sina,
        vector.x * sina + vector.y * cosa,
        vector.z
    )


class Vector(object):
    '''
    A basic 3D Vector class. Supports basic operations like addition, subtraction,
    scalar multiplication, dot product, cross product, normalization, and length
    calculation. Can be compared to other vectors for equality.
    '''
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
        if my_length == 0:
            return self
        else:
            return Vector(self.x / my_length, self.y / my_length, self.z / my_length)
    def __repr__(self):
        return f"Vector(x={self.x}, y={self.y}, z={self.z})"
    

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

        edges = []
        for v1, v2 in zip(self.vertices[:-1], self.vertices[1:]):
            edges.append(v2 - v1)
        edges.append(self.vertices[0] - self.vertices[-1])
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
        cross = self.edges[0].cross(point - self.vertices[0])
        for edge, edge_base in zip(self.edges[1:], self.vertices[1:]):
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


def test_pixel_in_screen_poly(row, col, screen_poly):
    ray_edge_intersection_count = 0
    for vertex, edge in zip(screen_poly.vertices, screen_poly.edges):
        if edge.y != 0:
            beta = (row - vertex.y) / edge.y
            alpha = beta * edge.x - (col - vertex.x)

            if alpha > 0 and 0 < beta < 1:
                ray_edge_intersection_count += 1
    
    if ray_edge_intersection_count % 2 == 0:
        return False
    else:
        return True


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

    player_pos = Vector(math.sqrt(2) / 2 * PLAYER_DIST, math.sqrt(2) / 2 * PLAYER_DIST, PLAYER_HEIGHT)
    player_dir = (Vector(0, 0, PLAYER_HEIGHT) - player_pos).normalize()

    screen_height, screen_width = screen.getmaxyx()
    center_screen_row, center_screen_col = screen_height // 2, screen_width // 2

    # TODO: Gotta do something with the pixel size to scale polys on the screen based on given FOV
    v_pix_size = 2 * SCREEN_DISTANCE * math.tan(VFOV / 2) / screen_height
    h_pix_size = 2 * SCREEN_DISTANCE * math.tan(HFOV / 2) / screen_width

    light_location = Vector(0.5, LIGHT_DISTANCE, 0.5)

    # Define a translation from world origin to screen center, and screen center to TLC
    T_origin_screen = Vector(
        0, 
        -PLAYER_HEIGHT, 
        -(PLAYER_DIST - SCREEN_DISTANCE)
    )
    T_origin_screen = T_origin_screen * -1  # Active v. passive translation

    T_screen_center_to_tlc = Vector(
        center_screen_col,
        center_screen_row,
        0
    )

    player_in_screen_coords = Vector(0, 0, -SCREEN_DISTANCE)

    n = 0
    while True: # n < 10:
        n += 1
        
        t0 = time.time()

        screen_buffer = [[' '] * (screen_width - 1) for _ in range(screen_height - 1)]

        # Define a rotation matrix from world coords to screen coords
        screen_z_hat = player_dir
        screen_y_hat = Vector(0, 0, -1)
        screen_x_hat = screen_y_hat.cross(screen_z_hat)
        # R_world_screen = [  # Active transformation
        #     Vector(screen_x_hat.x, screen_y_hat.x, screen_z_hat.x), 
        #     Vector(screen_x_hat.y, screen_y_hat.y, screen_z_hat.y),
        #     Vector(screen_x_hat.z, screen_y_hat.z, screen_z_hat.z)
        # ]
        R_world_screen = [  # Passive transformation (Transpose of active, also inverse since R is unitary)
            screen_x_hat,
            screen_y_hat,
            screen_z_hat
        ]

        polys_to_display = []
        min_row, max_row, min_col, max_col = 2 * screen_height, -1, 2 * screen_width, -1
        for poly_num, poly in enumerate(cube_polys):
            if poly.surface_vector.dot(player_dir) >= 0:
                continue

            trans_vertices = []
            any_in_screen = False
            for vertex in poly.vertices:

                # For starting screen orientation, poly faces 3 and 5 should be the only two that show. That is working.
                # poly face 3, vertex 0 (1, 0, 0) should project to something like 
                #   (-0.707, player_height, player_dist - screen_dist - 0.707)

                # Rotate into screen coords
                v_3Dscreen_coords = Vector(*[R_row.dot(vertex) for R_row in R_world_screen])
                # Translate origin of screen coords
                v_3Dscreen_coords = v_3Dscreen_coords + T_origin_screen
                # Project onto the screen plane (z_screen = 0)
                player_to_vertex = v_3Dscreen_coords - player_in_screen_coords
                v_in_screen = Vector(player_to_vertex.x / h_pix_size, player_to_vertex.y / v_pix_size, 0)
                # Shift 0,0 from center of screen to TLC
                v_in_screen = v_in_screen + T_screen_center_to_tlc
                trans_vertices.append(v_in_screen)
                # Check to see if at least one vertex is on screen
                if (screen_width - 2) >= v_in_screen.x >= 0 and (screen_height - 2) >= v_in_screen.y >= 0:
                    any_in_screen = True
            
            if any_in_screen:

                for vertex in trans_vertices:
                    if int(vertex.x + 0.5) < min_col:
                        min_col = max([int(vertex.x), 0])
                    if int(vertex.y + 0.5) < min_row:
                        min_row = max([int(vertex.y), 0])
                    if int(vertex.x + 0.5) > max_col:
                        max_col = min([int(vertex.x), screen_width - 2])
                    if int(vertex.y + 0.5) < max_row:
                        max_row = min([int(vertex.y), screen_height - 2])

                polys_to_display.append((poly_num, Poly(trans_vertices)))

        # Loop over pixels that contain polys to draw
        poly_pixels = {}
        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):
                for poly_num, screen_poly in polys_to_display:
                    
                    pixel_selector = 0
                    
                    if test_pixel_in_screen_poly(row, col, screen_poly):
                        if poly_num in poly_pixels:
                            pixel_selector = poly_pixels[poly_num]
                        else:
                            # Determine brightness... 
                            # Factors... dist from face to light, ambient light, face to light ang, face to camera dist
                            poly = cube_polys[poly_num]
                            poly_center = Vector(0, 0, 0)
                            for vertex in poly:
                                poly_center = poly_center + (vertex * 0.25)
                            
                            # player_to_poly_distance = (poly_center - player_pos).length()
                            light_to_poly = light_location - poly_center
                            # light_to_poly_distance = light_to_poly.length()
                            light_to_poly_cos_angle = light_to_poly.normalize().dot(poly.surface_vector)

                            if light_to_poly_cos_angle > 0:
                                light_reflection = (LIGHT_BRIGHTNESS * light_to_poly_cos_angle) # - 2 * light_to_poly_distance
                            else:
                                light_reflection = 0

                            viz_brightness = (
                                light_reflection + AMBIENT_LIGHT # - 2 * player_to_poly_distance
                            )
                            viz_brightness = min([viz_brightness, SATURATION_LEVEL - SATURATION_LEVEL / 1000])

                            pixel_selector = max([0, int((viz_brightness / SATURATION_LEVEL) * len(GRAYS))])

                            poly_pixels[poly_num] = pixel_selector

                        screen_buffer[row][col] = GRAYS[pixel_selector]
            
        player_pos = z_rotate(player_pos, PLAYER_ANGULAR_VELOCITY)
        player_dir = z_rotate(player_dir, PLAYER_ANGULAR_VELOCITY)
        
        buffer_as_string = '\n'.join([''.join(row) for row in screen_buffer])
        screen.addstr(0, 0, buffer_as_string)
        
        t = time.time()
        if (1 / (t - t0)) > REFRESH_RATE:
            time.sleep((1 / REFRESH_RATE) - (t - t0))

        t = time.time()
        fps = 1 / (t - t0)

        screen.addstr(0, 0, f'{fps:.4f}')
        screen.refresh()
                    

if __name__ == '__main__':

    curses.wrapper(main)
