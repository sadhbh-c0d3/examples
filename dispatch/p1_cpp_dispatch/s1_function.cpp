int area_square(int side) {
    return side * side;
}

int area_triangle(int side, int height) {
    return (side * height)/2;
}

int main() {
    auto a1 = area_square(5);
    auto a2 = area_triangle(10, 20);
    return a1 + a2;
}
