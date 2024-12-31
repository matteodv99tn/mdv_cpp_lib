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

struct Function {

    template <typename CallableFunction>
    Function(CallableFunction f) : 
        _fun(std::make_unique<FunctionModel<CallableFunction>>(std::move(f))) {}

    double operator()(const double& v) const { return (*_fun)(v); }
    std::string describe() const { return _fun->describe(); }

private:
    struct FunctionConcept {
        virtual double operator()(const double& v) const = 0;
        virtual std::string describe() const = 0;
    };

    template <typename CallableFunction>
    struct FunctionModel : FunctionConcept {
        explicit FunctionModel(CallableFunction fun): _f(std::move(fun)) {};

        double operator()(const double& v) const override { return _f(v); }
        std::string describe() const override { return _f.describe(); }

        CallableFunction _f;
    };

    std::unique_ptr<FunctionConcept> _fun;
};

/* Content extracted for explanation
struct FunctionConcept {
    virtual double operator()(const double& v) const = 0;
    virtual std::string describe() const = 0;
};

template <typename CallableFunction>
struct FunctionModel : FunctionConcept {
    explicit FunctionModel(CallableFunction fun): _f(std::move(fun)) {};

    double operator()(const double& v) const override { return _f(v); }
    std::string describe() const override { return _f.describe(); }

    CallableFunction _f;
};

struct Function {
   // ...
};
*/

using DrawRange = std::pair<double, double>;

struct Drawer {
    
    template <typename CallableDrawer>
    Drawer(CallableDrawer drawer) : 
        _d(std::make_unique<DrawerModel<CallableDrawer>>(std::move(drawer))) {}

    void draw(const Function& fun, DrawRange range = {0.0, 1.0}) { _d->draw(fun, range); }

private:
    struct DrawerConcept {
        virtual void draw(const Function& fun, DrawRange range) = 0;
    };

    template <typename CallableDrawer>
    struct DrawerModel : DrawerConcept {
        explicit DrawerModel(CallableDrawer drawer): _drawer(std::move(drawer)) {}

        void draw(const Function& fun, DrawRange range) override { return _drawer.draw(fun, range); }

        CallableDrawer _drawer;
    };

    std::unique_ptr<DrawerConcept> _d;
};

using FunctionCollection = std::vector<Function>;

void draw_functions(const FunctionCollection& funcs, Drawer drawer){
    for (const auto& f: funcs) drawer.draw(f);
}

struct VoidDrawer {
    void draw(const Function& fun, DrawRange range = {0.0, 1.0}) {
        fmt::println("Drawing function {}", fun.describe());
        fmt::println(" > Value at {}: {}", range.first, fun(range.first));
        fmt::println(" > Value at {}: {}", range.second, fun(range.second));
    }
};

int main() {
    FunctionCollection funcs;
    funcs.emplace_back(SqrtFunction());
    funcs.emplace_back(ParametricParabola(2.0, 0.0, 1.0));

    draw_functions(funcs, VoidDrawer());
}
