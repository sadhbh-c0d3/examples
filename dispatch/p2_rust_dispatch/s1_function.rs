pub fn square_area(side: i32) -> i32 {
    side * side
}

pub fn triangle_area(side: i32, height: i32) -> i32 {
    (side * height)/2
}

pub fn main() -> i32 {
    let v1 = square_area(5);
    let v2 = triangle_area(10, 20);
    v1 + v2
}
