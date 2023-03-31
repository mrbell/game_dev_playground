// Draw a cube to the terminal and rotate the camera around the cube at a fixed rate. 
// Porting from cube.py version written in pure python.

// Things to try once this is working:
// - Clean up code
// - Play with lighting (location, how brightness is computed, etc.)
// - Display using colors
// - Display using more character types, maybe super resolve?
// - Add more objects to scene
// - Add input loop so player controls rotation
// - Allow camera to pitch (in addition to yaw like it does now)
// - Allow player to walk around FPS style, maybe free fly to change height and pitch as well as x,y position and yaw
// - Add more lights
// - Create a little level to walk around in
// - Add textures
// - Add sprites

use std::f64::consts::PI;
use std::fmt;

const REFRESH_RATE: f64 = 60.0;
const GRAYS: [char; 10] = [' ', '.', ':', '-', '=', '+', '*', '#', '%', '@'];
const PLAYER_DISTANCE: f64 = 3.0;
const PLAYER_HEIGHT: f64 = 0.5;
const PLAYER_ANGULAR_VELOCITY: f64 = 2.0 * PI / 20.0 / REFRESH_RATE;
const SCREEN_DISTANCE: f64 = 0.5;
const HFOV: f64 = 80.0 * PI / 180.0;
const VFOV: f64 = 60.0 * PI / 180.0;
const LIGHT_DISTANCE: f64 = 5.0;
const LIGHT_BRIGHTENSS: f64 = 2.0;
const AMBIEINT_LIGHT: f64 = 1.0;
const SATURATION_LEVEL: f64 = 5.0;
const EPS: f64 = 1.0e-7;


#[derive(Debug)]
struct Vector {
    x: f64,
    y: f64,
    z: f64
}

impl Vector {

    fn length(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    fn normalize(&self) -> Vector {
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

    fn dot(&self, other: &Vector) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    fn cross(&self, other: &Vector) -> Vector {
        Vector {
            x: self.y * other.z - self.z * other.y,
            y: -self.x * other.z + self.z * other.x,
            z: self.x * other.y - self.y * other.x
        }
    }

    fn copy(&self) -> Vector {
        Vector{x: self.x, y: self.y, z: self.z}
    }

    fn add(&self, other: &Vector) -> Vector {
        Vector{
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z
        }
    }
    
    fn subtract(&self, other: &Vector) -> Vector {
        Vector{
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z
        }
    }

    fn scale(&self, factor: f64) -> Vector {
        Vector {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor
        }
    }

    fn equals(&self, other: &Vector) -> bool {
        if (self.x == other.x) & (self.y == other.y) & (self.z == other.z) {
            true
        } else {
            false
        }
    }
}

impl fmt::Display for Vector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vector {{ x: {}, y: {}, z: {} }}", self.x, self.y, self.z)
    }

}


fn z_rotate(v: &Vector, angle_rad: f64) -> Vector {
    let cosa= angle_rad.cos();
    let sina = angle_rad.sin();

    Vector{
        x: v.x * cosa - v.y * sina,
        y: v.x * sina + v.y * cosa,
        z: v.z
    } 
}


fn main() {

    let angle: f64 = PI / 2.0;
    let vector = Vector {
        x: 1.0,
        y: 0.0,
        z: 0.0
    };

    let result = z_rotate(&vector, angle);
    println!("**ROTATION**");
    println!("Vector length {}", vector.length());
    println!("Result is {:?}", result);
    println!("Result length {}", result.length());

    let other_vector = Vector {x: 2.0, y: 0.0, z: 0.0};
    println!("**NORM**");
    println!("Vector length {}", other_vector.length());
    println!("Vector length {}", other_vector.normalize().length());

    println!("**DOT AND CROSS**");
    println!("Dot result is {}", other_vector.dot(&vector));
    println!("Cross result is {:?}", other_vector.cross(&vector));

    println!("**COPY**");
    println!("Result is {:?}", other_vector.copy());

    println!("**ADD, SUB, and SCALE**");
    println!("Result is {:?}", vector.add(&other_vector));
    println!("Result is {:?}", vector.subtract(&other_vector));
    println!("Result is {:?}", vector.scale(6.0));

    println!("**EQUALS**");
    println!("Result is {}", vector.equals(&other_vector));
    println!("Result is {}", vector.equals(&vector.copy()));

    println!("**DISPLAY**");
    println!("{vector}")  // Probably just prefer the Debug
}
