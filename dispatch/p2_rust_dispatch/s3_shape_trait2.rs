#[derive(Clone, Copy)]
struct Square {
    side: i32
}

impl Square {
    pub fn new(side: i32) -> Self {
        Self { side }
    }
}

#[derive(Clone, Copy)]
struct Triangle {
    side: i32,
    height: i32
}

impl Triangle {
    pub fn new(side: i32, height: i32) -> Self {
        Self { side, height }
    }
}

pub trait Shape {
    fn area(&self) -> i32;
}

impl Shape for Square {
    fn area(&self) -> i32 {
        self.side * self.side
    }
}

impl Shape for Triangle {
    fn area(&self) -> i32 {
        (self.side * self.height)/2
    }
}

pub fn total_area_static(shapes: &[impl Shape]) -> i32 {
    shapes.iter().map(|x| x.area()).sum()
}

pub fn total_area_dynamic(shapes: &[&dyn Shape]) -> i32 {
    shapes.iter().map(|x| x.area()).sum()
}

pub fn run_static() -> i32 {
    let v1 = Square::new(5);
    let v2 = Triangle::new(10, 20);
    total_area_static(&[v1, v1]) + total_area_static(&[v2, v2])
}

pub fn run_dynamic() -> i32 {
    let v1 = Square::new(5);
    let v2 = Triangle::new(10, 20);
    total_area_dynamic(&[&v1, &v2])
}

pub fn main() -> i32 {
    run_static() + run_dynamic()
}
