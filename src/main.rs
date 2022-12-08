extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate piston;
extern crate vecmath;
extern crate rand;

use glutin_window::GlutinWindow as Window;
use graphics::{rectangle, color};
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop::{EventSettings, Events};
use piston::input::{RenderArgs, RenderEvent, UpdateArgs, UpdateEvent};
use piston::window::WindowSettings;
use vecmath::*;


const HEIGHT: u32 = 1000;
const WIDTH: u32 = 1500;
const MAX_FORCE: f64 = 0.09;
const MAX_SPEED: f64 =1.5;
const PERCEPTION: f64 = 50.0;


//The boid structure
#[derive(Clone, Copy)]
struct Boid {
    location: Vector2<f64>,
    velocity: Vector2<f64>,
    acceleration: Vector2<f64>,
}

//Function to limit the magnitude of a giving vector by a given magnitude
//Returns the same vector if the max magnitude is not exceeded
//Otherwise returns the max magnitude.
pub fn limit_mag(vector: Vector2<f64>, limiter: f64) -> Vector2<f64>{
    if vec2_len(vector) > limiter {
        return vec2_scale(vec2_normalized(vector), limiter);
    }
    return vector;
}

//Function to set a new magnitude of a given vector
//Returns a new vector
pub fn set_mag(vector: Vector2<f64>, mag: f64) -> Vector2<f64>{
    let mut new_vec = vector;
    let magg = vec2_len(vector);
    new_vec[0] = new_vec[0]*(mag/magg);
    new_vec[1] = new_vec[1]*(mag/magg);
    return new_vec;

}

impl Boid {
    pub fn new() -> Boid{
        Boid {
            location: [rand::random::<f64>()*WIDTH as f64, rand::random::<f64>()*HEIGHT as f64],
            velocity: [rand::random(),rand::random()],
            acceleration: [0.0,0.0],
        }
    }
    //Align algoritm, takes the close boids by a given range and calculates the avarage direction
    //Then returns the avarage direction in form of a vector where the magnitude is limited to the MAX_FORCE
    pub fn align(&mut self,boids: Vec<Boid>) -> Vector2<f64>{
        let mut avg: Vector2<f64> = [0.0,0.0];
        let mut total = 0;
        
        for boid in boids.iter(){
            let distance = vec2_len(vec2_sub(boid.location, self.location));
            if distance < PERCEPTION && boid.location != self.location{
                avg = vec2_add(avg, boid.velocity);
                total = total + 1;
            }
            
        }
        if total > 0{
            //Calculate the desired direction to move based on the avarage direction.
            avg = vec2_scale(avg, 1.0/(total as f64));
            avg = set_mag(avg, MAX_SPEED);
            avg = vec2_sub(avg, self.velocity);
            avg = limit_mag(avg, MAX_FORCE);
            
            return avg;
        }
        return [0.0,0.0]
    }
    //Chohesion algoritm, calculates the avarage position by the closest boids given by the perception range
    //Returns a desired vector to move for the boid
    pub fn cohesion(&mut self, boids: Vec<Boid>) -> Vector2<f64>{
        let mut avg: Vector2<f64> = [0.0,0.0];
        let mut total = 0;
        //Calculate the avarage position in the perception range
        for boid in boids.iter(){
            let distance = vec2_len(vec2_sub(boid.location, self.location));
            if distance < PERCEPTION && boid.location != self.location{
                avg = vec2_add(avg, boid.location);
                total += 1;
            }
            
            
        }
        if total > 0{
            //Calculate the desired direction to move based on the avarage position
            avg = vec2_scale(avg, 1.0/(total as f64));
            avg = vec2_sub(avg, self.location);
            
            avg = set_mag(avg, MAX_SPEED);
            avg = vec2_sub(avg, self.velocity);
            avg = limit_mag(avg, MAX_FORCE);
            //println!("Distance: {}, avg: {}", vec2_len(vec2_sub(self.location, self.test)), vec2_len(avg));
            return avg;
        }
        return [0.0,0.0]
    }
    //Calculates the avarage vector to prevent the boid steering into other boids
    //Returns the desired direction to move.
    pub fn seperation(&mut self, boids: Vec<Boid>) -> Vector2<f64>{
        let mut avg: Vector2<f64> = [0.0,0.0];
        let mut total = 0;
        //Calculate the avarage vector for the boids within the perception range
        //Make the vector propotional to the distance between every boid. The close the stronger.
        for boid in boids.iter(){
            let distance = vec2_len(vec2_sub(boid.location, self.location));
            if distance < PERCEPTION && boid.location != self.location{
                let mut diff = vec2_sub(self.location, boid.location);
                diff = vec2_scale(diff, 1.0/distance);
                avg = vec2_add(avg, diff);
                total += 1;
            }
            
            
        }
        //Return the desired direction in a vector.
        if total > 0{
            avg = vec2_scale(avg, 1.0/(total as f64));
            avg = set_mag(avg, MAX_SPEED);
            avg = vec2_sub(avg, self.velocity);
            avg = limit_mag(avg, MAX_FORCE);
            return avg;
        }
        return [0.0,0.0]
    }
    //Create a rectangle for the boid.
    pub fn rect(&self) -> [f64; 4]{
        rectangle::square(0.0, 0.0, 8.0)
    }
    //Function to calculate the new acceleration for the boid
    //This is done by summing all the return vectors from cohesion, seperation and alignment
    //The summed vector is now the desired accelaration to move with.
    pub fn flock(&mut self, boids: Vec<Boid>){
        //Reset the acceleration for each update
        self.acceleration = [0.0, 0.0];
        //Gather the desired directions to move witj
        let alignment = self.align(boids.to_vec());
        let cohesion = self.cohesion(boids.to_vec());
        let mut seperation = self.seperation(boids);

        //Scale the seperation vector down to prevent too much seperation
        seperation = vec2_scale(seperation, 0.90);
        //Sum all the desired direction and apply it to the acceleration.
        self.acceleration = vec2_add(cohesion, self.acceleration);
        self.acceleration = vec2_add(self.acceleration, alignment);
        self.acceleration = vec2_add(self.acceleration, seperation);
    }
    //The update function for the boid
    pub fn update(&mut self, boids: Vec<Boid>) {
        //The location is based on the velocity, and the velocity is based on the acceleration.
        self.flock(boids);
        self.location = vec2_add(self.location, self.velocity);
        self.velocity = vec2_add(self.velocity, self.acceleration);
        self.velocity = limit_mag(self.velocity, MAX_SPEED);
        
        //Prevent the boid from leaving the visual window of the program.
        if self.location[0] > WIDTH as f64 {
            self.location[0] = 0.0;
        }
        if self.location[0] < 0.0 {
            self.location[0] = WIDTH as f64
        }
        if self.location[1] > HEIGHT as f64 {
            self.location[1] = 0.0;
        }
        if self.location[1] < 0.0 {
            self.location[1] = HEIGHT as f64;
        }
    }
}

pub struct App {
    gl: GlGraphics,
    boids: Vec<Boid>
}

impl App {
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;
        let boids = &self.boids;
        self.gl.draw(args.viewport(), |c, gl| {
            // Clear the screen.
            clear(color::BLACK, gl);
            for boid in boids {
                let transform = c.transform.trans(boid.location[0], boid.location[1]);
                rectangle(color::WHITE, boid.rect(), transform, gl)
            }
        });
    }

    fn update(&mut self, args: &UpdateArgs) {
        // Rotate 2 radians per second.
        let boids = self.boids.to_vec();
        for boid in self.boids.iter_mut() {
            boid.update(boids.to_vec());
        }
    }
}

fn main() {
    // Change this to OpenGL::V2_1 if not working.
    let opengl = OpenGL::V3_2;

    // Create a Glutin window.
    let mut window: Window = WindowSettings::new("spinning-square", [WIDTH, HEIGHT])
        .graphics_api(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    // Create a new game and run it.

    let mut boid_box: Vec<Boid> = vec![];
    for i in 0..300{
        boid_box.push(Boid::new())
    }

    let mut app = App {
        gl: GlGraphics::new(opengl),
        boids: boid_box,
    };

    let mut events = Events::new(EventSettings::new());
    while let Some(e) = events.next(&mut window) {
        if let Some(args) = e.render_args() {
            app.render(&args);
        }

        if let Some(args) = e.update_args() {
            app.update(&args);
        }
    }
}
