#ifndef MDV_DMP_COORDINATE_SYSTEM_INTERFACE_HPP
#define MDV_DMP_COORDINATE_SYSTEM_INTERFACE_HPP

#include <Eigen/Dense>
#include <vector>

#include <mdv/macros.hpp>

namespace mdv::dmp {

template <typename CsImplementation>
class CoordinateSystemBase {
public:
    CoordinateSystemBase(const double* tau) : _tau(tau) {}

    double
    operator()(const double& t) const {
        return cs().eval(t);
    }

    std::vector<double>
    operator()(const std::vector<double>& ts) const {
        std::vector<double> res;
        res.reserve(ts.size());
        for (const auto& t : ts) res.emplace_back(cs().eval(t));
        return res;
    }

    Eigen::VectorXd
    operator()(const Eigen::VectorXd& ts) const {
        Eigen::VectorXd res(ts.size());
        double*         res_it = res.data();
        for (const auto& t : ts) *(res_it++) = cs().eval(t);
        return res;
    }

protected:
    MDV_NODISCARD const double&
    tau() const {
        assert(_tau);
        return *_tau;
    }

private:
    // clang-format off
    using CoordSys = CsImplementation;
    CoordSys&       cs()       { return static_cast<CoordSys&>(*this); }
    const CoordSys& cs() const { return static_cast<const CoordSys&>(*this); }

    // clang-format on

    const double* _tau = nullptr;
};


}  // namespace mdv::dmp


#endif  // MDV_DMP_COORDINATE_SYSTEM_INTERFACE_HPP
