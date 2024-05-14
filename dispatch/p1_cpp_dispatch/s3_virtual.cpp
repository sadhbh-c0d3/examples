struct Shape {
    virtual int area() const = 0;
};

struct Square : Shape {
    virtual int area() const override { return side * side; }
    Square(int side): side{side} {}
    int side;
};

struct Triangle : Shape {
    virtual int area() const override { return (side * height)/2; }
    Triangle(int side, int height): side{side}, height{height} {}
    int side;
    int height;
};

int run_static() {
    Square s{5};
    Triangle t{10, 20};
    return s.area() + t.area();
}

int total_area1(Shape &s1, Shape &s2) {
    return s1.area() + s2.area();
}

int total_area2(Shape const **shapes, int count) {
    int v = 0;
    while (count--) {
        v += (*shapes)->area();
        ++shapes;
    }
    return v;
}

int run_dynamic1() {
    Square s{5};
    Triangle t{10, 20};
    return total_area1(s, t);
}

int run_dynamic2() {
    Square s{5};
    Triangle t{10, 20};
    Shape const *shapes[] = {&s, &t};
    return total_area2(shapes, 2);
}

int main() {
    auto v1 = run_static();
    auto v2 = run_dynamic1();
    auto v3 = run_dynamic2();
    return v1 + v2 + v3;
}