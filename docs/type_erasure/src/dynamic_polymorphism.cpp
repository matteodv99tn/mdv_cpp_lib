// clang-format off
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <fmt/os.h>

struct Function {
    virtual double operator()(const double& x) const = 0;
    virtual std::string describe() const = 0;
};

struct Drawer {
    using DrawRange = std::pair<double, double>;
    virtual void draw(const Function* fun, DrawRange range = {0.0, 1.0}) = 0;
};


struct SqrtFunction : Function {
    double operator()(const double& x ) const override { return std::sqrt(x); }
    std::string describe() const override { return "square_root"; }
};

struct ParametricParabola: Function {
    ParametricParabola(double aa, double bb, double cc): a(aa), b(bb), c(cc) {}
    double operator()(const double& x ) const override { return a*x*x + b*x + c; }
    std::string describe() const override { return "parametric_parabola"; }
    double a{1.0};
    double b{-1.0};
    double c{0.0};
};

struct VoidDrawer: Drawer{
    void draw(const Function* fun, DrawRange range = {0.0, 1.0}) override {
        fmt::println("Drawing function {}", fun->describe());
        fmt::println(" > Value at {}: {}", range.first, (*fun)(range.first));
        fmt::println(" > Value at {}: {}", range.second, (*fun)(range.second));
    }
};


using FunctionPtr = std::unique_ptr<Function>;
using FunctionCollection = std::vector<FunctionPtr>;

void draw_functions(const FunctionCollection& funcs, Drawer* drawer){
    for (const auto& f: funcs) drawer->draw(f.get());
}

int main() {
    FunctionCollection funcs;
    funcs.push_back(std::make_unique<SqrtFunction>());
    funcs.push_back(std::make_unique<ParametricParabola>(2.0, 0.0, 1.0));

    VoidDrawer drawer;
    draw_functions(funcs, &drawer);
}
