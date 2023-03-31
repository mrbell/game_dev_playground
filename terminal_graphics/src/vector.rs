use std::f64::consts::PI;
use std::fmt;


#[derive(Debug)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64
}

impl Vector {

    pub fn length(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    pub fn normalize(&self) -> Vector {
        let mut my_length = self.length();
        if my_length < 1.0e-12 {
            my_length = 1.0;
        }
        Vector {
            x: self.x / my_length,
            y: self.y / my_length,
            z: self.z / my_length
        }
    }

    pub fn dot(&self, other: &Vector) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Vector) -> Vector {
        Vector {
            x: self.y * other.z - self.z * other.y,
            y: -self.x * other.z + self.z * other.x,
            z: self.x * other.y - self.y * other.x
        }
    }

    pub fn copy(&self) -> Vector {
        Vector{x: self.x, y: self.y, z: self.z}
    }

    pub fn add(&self, other: &Vector) -> Vector {
        Vector{
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z
        }
    }
    
    pub fn subtract(&self, other: &Vector) -> Vector {
        Vector{
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z
        }
    }

    pub fn scale(&self, factor: f64) -> Vector {
        Vector {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor
        }
    }

    pub fn equals(&self, other: &Vector) -> bool {
        if (self.x == other.x) & (self.y == other.y) & (self.z == other.z) {
            true
        } else {
            false
        }
    }
}

impl fmt::Display for Vector {
    pub fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vector {{ x: {}, y: {}, z: {} }}", self.x, self.y, self.z)
    }

}


pub fn z_rotate(v: &Vector, angle_rad: f64) -> Vector {
    let cosa= angle_rad.cos();
    let sina = angle_rad.sin();

    Vector{
        x: v.x * cosa - v.y * sina,
        y: v.x * sina + v.y * cosa,
        z: v.z
    } 
}
