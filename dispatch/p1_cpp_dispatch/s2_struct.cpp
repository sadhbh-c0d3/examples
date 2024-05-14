struct Square {
    int side;
};

struct Triangle {
    int side;
    int height;
};

int area_square(Square s) {
    return s.side * s.side;
}

int area_triangle(Triangle t) {
    return (t.side * t.height)/2;
}

int main() {
    auto a1 = area_square({5});
    auto a2 = area_triangle({10, 20});
    return a1 + a2;
}
