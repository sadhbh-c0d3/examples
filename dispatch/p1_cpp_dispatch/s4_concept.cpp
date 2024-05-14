#include <concepts>
#include <type_traits>

struct SquareImpl {
    int side;
};

struct TriangleImpl {
    int side;
    int height;
};

template<class T>
concept Square = requires(T x) {
    { x.side } -> std::convertible_to<int>;
};

template<class T>
concept Triangle = requires(T x) {
    { x.side } -> std::convertible_to<int>;
    { x.height } -> std::convertible_to<int>;
};

int area_square(Square auto &&s) {
    return s.side * s.side;
}

int area_triangle(Triangle auto &&t) {
    return (t.side * t.height)/2;
}

int main() {
    auto a1 = area_square<SquareImpl>({5});
    auto a2 = area_triangle<TriangleImpl>({10, 20});
    return a1 + a2;
}