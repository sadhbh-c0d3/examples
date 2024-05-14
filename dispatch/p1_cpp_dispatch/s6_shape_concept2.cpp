#include <concepts>
#include <type_traits>

struct SquareImpl {
    int side;
};

bool is_square(SquareImpl);

struct TriangleImpl {
    int side;
    int height;
};

bool is_triangle(TriangleImpl);

template<class T>
concept Square = requires(T x) {
    { x.side } -> std::convertible_to<int>;
    { is_square(x) } -> std::convertible_to<bool>;
};

template<class T>
concept Triangle = requires(T x) {
    { x.side } -> std::convertible_to<int>;
    { x.height } -> std::convertible_to<int>;
    { is_triangle(x) } -> std::convertible_to<bool>;
};

template<class T> struct area_traits {};

template<Square T> struct area_traits<T> {
    static int area(const T &x) { return x.side * x.side; }
};

template<Triangle T> struct area_traits<T> {
    static int area(const T &x) { return (x.side * x.height)/2; }
};

template<class T>
concept Shape = requires(const T &x) {
    { area_traits<T>::area(x) } -> std::convertible_to<int>;
};

int total_area() {
    return 0;
}

template<Shape T1, Shape ...T> 
int total_area(const T1 &x1, const T &...x) {
    return area_traits<std::remove_cvref_t<T1>>::area(x1) + total_area(x...);
}

int run_traits() {
    SquareImpl s{5};
    TriangleImpl t{10, 20};
    auto v1 = area_traits<SquareImpl>::area(s);
    auto v2 = area_traits<TriangleImpl>::area(t);
    return v1 + v2;
}

int run_total_area() {
    SquareImpl s{5};
    TriangleImpl t{10, 20};
    return total_area(s, t);
}

int main() {
    auto v1 = run_traits();
    auto v2 = run_total_area();
    return v1 + v2;
}
