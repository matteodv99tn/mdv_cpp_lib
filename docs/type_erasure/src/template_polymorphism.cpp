// clang-format off
#include <cmath>
#include <memory>
#include <string>
#include <variant>
#include <vector>
#include <fmt/os.h>


struct SqrtFunction {
    double operator()(const double& x ) const { return std::sqrt(x); }
    std::string describe() const { return "square_root"; }
};

struct ParametricParabola {
    ParametricParabola(double aa, double bb, double cc): a(aa), b(bb), c(cc) {}
    double operator()(const double& x ) const { return a*x*x + b*x + c; }
    std::string describe() const { return "parametric_parabola"; }
    double a{1.0};
    double b{-1.0};
    double c{0.0};
};

using DrawRange = std::pair<double, double>;

struct VoidDrawer {
    template <typename Function>
    void draw(const Function& fun, DrawRange range = {0.0, 1.0}) {
        fmt::println("Drawing function {}", fun.describe());
        fmt::println(" > Value at {}: {}", range.first, fun(range.first));
        fmt::println(" > Value at {}: {}", range.second, fun(range.second));
    }
};


/* What we want, but that doesn't work:
template <typename Drawer>
void draw_functions(const FunctionCollection& funcs, Drawer& drawer){
    for (const auto& f: funcs) drawer.draw(f);
}
*/

using AllFunctions = std::variant<SqrtFunction, ParametricParabola>;
using FunctionCollection = std::vector<AllFunctions>;

template <typename Drawer>
void draw_functions(const FunctionCollection& funcs, Drawer& drawer){
    for (const auto& f: funcs) 
        std::visit([&drawer](const auto& fun) { drawer.draw(fun); }, f);
}

int main() {
    FunctionCollection funcs;
    funcs.emplace_back(SqrtFunction());
    funcs.emplace_back(ParametricParabola(2.0, 0.0, 1.0));

    VoidDrawer drawer;
    draw_functions(funcs, drawer);
}
