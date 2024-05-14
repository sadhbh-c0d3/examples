#include <concepts>
#include <type_traits>

struct Square {
    int side;
    int area() const {
        return side * side;
    }
};

struct Triangle {
    int side;
    int height;
    int area() const {
        return (side * height)/2;
    }
};

template<class T>
concept Shape = requires(const T &cx) {
    { cx.area() } -> std::convertible_to<int>;
};

int total_area() {
    return 0;
}

template<Shape T1, Shape ...T> 
int total_area(const T1 x1, const T ...x) {
    return x1.area() + total_area(x...);
}

int main() {
    Square s{5};
    Triangle t{10, 20};
    return total_area(s, t);
}