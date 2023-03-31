use crate::vector::Vector;
use crate::vector::z_rotate;


#[derive(Debug)]
pub struct Poly {
    pub vertices: Vec<Vector>,
    pub edges: Vec<Vector>,
    pub surface_vector: Vector 
}


impl Poly {

    pub fn new(vertices: Vec<Vector>) -> Poly {
        
        let mut edges = Vec::<Vector>::new();
        for (v1, v2) in vertices[..vertices.len()-1].iter().zip(vertices[1..].iter()) {
            edges.push(v1.subtract(&v2));
        }
        edges.push(vertices[0].subtract(&vertices[vertices.len() - 1]));
        
        let surface_vector = edges[0].cross(&edges[1]);
        
        Poly {
            vertices: vertices,
            edges: edges,
            surface_vector: surface_vector
        }
    
    }

    pub fn add(&self, vector: &Vector) -> Poly {
        let mut new_vertices = Vec::<Vector>::new();
        for v in &self.vertices {
            new_vertices.push(v.add(&vector));
        }
        Poly::new(new_vertices)
    }

    pub fn contains_point(&self, point: &Vector) -> bool {
        let cross = self.edges[0].cross(&point.subtract(&self.vertices[0]));

        for (edge, edge_base) in self.edges[1..].iter().zip(self.vertices[1..].iter()) {
            let to_point = point.subtract(&edge_base);
            if cross.dot(&edge.cross(&to_point)) <= 0.0 {
                return false;
            }
        }
        true
    }

    pub fn project_ray_onto_plane(&self, base: &Vector, unit_vec: &Vector) -> Option<Vector> {
        let denom = self.surface_vector.dot(unit_vec);
        if denom.abs() < 1.0e-12 {
            return None;
        }
        let param = self.surface_vector.dot(&self.vertices[0].subtract(&base)) / denom;
        if param < 0.0 {
            return None;
        }
        Some(base.add(&unit_vec.scale(param)))
    }

    pub fn ray_intersection(&self, ray_base: &Vector, ray_unit_vec: &Vector) -> Option<Vector> {
        let intersection = self.project_ray_onto_plane(&ray_base, &ray_unit_vec);
        if intersection.is_none() {
            return None;
        }
        let intersection = intersection.unwrap();
        if self.contains_point(&intersection) {
            Some(intersection)
        } else {
            None
        }
    }

}


pub fn test_pixel_in_screen_poly(row: usize, col: usize, screen_poly: &Poly) -> bool {
    let mut ray_edge_intersection_count = 0;

    for (vertex, edge) in screen_poly.vertices.iter().zip(screen_poly.edges.iter()) {
        if edge.y != 0.0 {
            let beta = (row as f64 - vertex.y) / edge.y;
            let alpha = beta * edge.x - (col as f64 - vertex.x);
            if alpha > 0.0 && beta > 0.0 && beta < 1.0 {
                ray_edge_intersection_count += 1;
            }
        }
    }

    if ray_edge_intersection_count % 2 == 0 {
        return false;
    }
    true

}