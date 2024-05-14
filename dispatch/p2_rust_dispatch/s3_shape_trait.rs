struct Square {
    side: i32
}

impl Square {
    pub fn new(side: i32) -> Self {
        Self { side }
    }
}

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

pub fn main() -> i32 {
    let v1 = Square::new(5);
    let v2 = Triangle::new(10, 20);
    v1.area() + v2.area()
}
