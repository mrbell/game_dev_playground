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
use crate::vector::Vector;
use crate::vector::z_rotate;

mod vector;

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
