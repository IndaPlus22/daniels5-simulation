extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate piston;
extern crate vecmath;
extern crate rand;

use glutin_window::GlutinWindow as Window;
use graphics::{circle_arc, rectangle, color, CircleArc};
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop::{EventSettings, Events};
use piston::input::{RenderArgs, RenderEvent, UpdateArgs, UpdateEvent};
use piston::window::WindowSettings;
use vecmath::*;
use rand::*;


const HEIGHT: u32 = 1000;
const WIDTH: u32 = 1000;
const MAX_FORCE: f64 = 0.01;
const MAX_SPEED: f64 =1.0;
const PERCEPTION: f64 = 50.0;

#[derive(Clone, Copy)]
struct Boid {
    location: Vector2<f64>,
    velocity: Vector2<f64>,
    acceleration: Vector2<f64>,
    test: Vector2<f64>
}


pub fn limitMag(vector: Vector2<f64>, limiter: f64) -> Vector2<f64>{
    if(vec2_len(vector) > limiter){
        return vec2_scale(vec2_normalized(vector), limiter);
    }
    return vector;
}

pub fn setMag(vector: Vector2<f64>, mag: f64) -> Vector2<f64>{
    let mut newVec = vector;
    let magg = vec2_len(vector);
    newVec[0] = newVec[0]*(mag/magg);
    newVec[1] = newVec[1]*(mag/magg);
    return newVec;

}

impl Boid {
    pub fn new() -> Boid{
        Boid {
            location: [rand::random::<f64>()*WIDTH as f64, rand::random::<f64>()*HEIGHT as f64],
            velocity: [rand::random(),rand::random()],
            acceleration: [0.0,0.0],
            test: [0.0,0.0]
        }
    }

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
            avg = vec2_scale(avg, 1.0/(total as f64));
            avg = setMag(avg, MAX_SPEED);
            avg = vec2_sub(avg, self.velocity);
            avg = limitMag(avg, MAX_FORCE);
            
            return avg;
        }
        return [0.0,0.0]
    }

    pub fn cohesion(&mut self, boids: Vec<Boid>) -> Vector2<f64>{
        let mut avg: Vector2<f64> = [0.0,0.0];
        let mut total = 0;
        
        for boid in boids.iter(){
            let distance = vec2_len(vec2_sub(boid.location, self.location));
            if distance < PERCEPTION && boid.location != self.location{
                avg = vec2_add(avg, boid.location);
                total += 1;
            }
            
            
        }
        if total > 0{
            avg = vec2_scale(avg, 1.0/(total as f64));
            self.test = avg;
            avg = vec2_sub(avg, self.location);
            
            avg = setMag(avg, MAX_SPEED);
            avg = vec2_sub(avg, self.velocity);
            avg = limitMag(avg, MAX_FORCE);
            //println!("Distance: {}, avg: {}", vec2_len(vec2_sub(self.location, self.test)), vec2_len(avg));
            return avg;
        }
        return [0.0,0.0]
    }

    pub fn seperation(&mut self, boids: Vec<Boid>) -> Vector2<f64>{
        let mut avg: Vector2<f64> = [0.0,0.0];
        let mut total = 0;
        
        for boid in boids.iter(){
            let distance = vec2_len(vec2_sub(boid.location, self.location));
            if distance < PERCEPTION && boid.location != self.location{
                let mut diff = vec2_sub(self.location, boid.location);
                diff = vec2_scale(diff, 1.0/distance);
                avg = vec2_add(avg, diff);
                total += 1;
            }
            
            
        }
        if total > 0{
            avg = vec2_scale(avg, 1.0/(total as f64));
            self.test = avg;
            avg = setMag(avg, MAX_SPEED);
            avg = vec2_sub(avg, self.velocity);
            avg = limitMag(avg, MAX_FORCE);
            //println!("Distance: {}, avg: {}", vec2_len(vec2_sub(self.location, self.test)), vec2_len(avg));
            return avg;
        }
        return [0.0,0.0]
    }

    pub fn rect(&self) -> [f64; 4]{
        rectangle::square(0.0, 0.0, 8.0)
    }

    pub fn flock(&mut self, boids: Vec<Boid>){
        self.acceleration = [0.0, 0.0];
        let alignment = self.align(boids.to_vec());
        let cohesion = self.cohesion(boids.to_vec());
        let mut seperation = self.seperation(boids);


        seperation = vec2_scale(seperation, 0.9);

        self.acceleration = vec2_add(cohesion, self.acceleration);
        self.acceleration = vec2_add(self.acceleration, alignment);
        self.acceleration = vec2_add(self.acceleration, seperation);

        //self.acceleration = cohesion;
    }

    pub fn update(&mut self, boids: Vec<Boid>) {
        self.flock(boids);
        self.location = vec2_add(self.location, self.velocity);
        self.velocity = vec2_add(self.velocity, self.acceleration);
        self.velocity = limitMag(self.velocity, MAX_SPEED);
        //println!("{}", self.velocity[0]);
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
    gl: GlGraphics, // OpenGL drawing backend.
    rotation: f64,  // Rotation for the square.
    boids: Vec<Boid>
}

impl App {
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
        const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
        let boids = &self.boids;
        self.gl.draw(args.viewport(), |c, gl| {
            // Clear the screen.
            clear(GREEN, gl);
            // Draw a box rotating around the middle of the screen.
            for boid in boids {
                let transform = c.transform.trans(boid.location[0], boid.location[1]);
                rectangle(RED, boid.rect(), transform, gl)
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
    let test: Vector2<f64> = [5.0,5.0];
    println!("{}", vec2_len(test));
    println!("{}", vec2_len(setMag(test, 10.0)));
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
    for i in 0..200{
        boid_box.push(Boid::new())
    }

    let mut app = App {
        gl: GlGraphics::new(opengl),
        rotation: 0.0,
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
