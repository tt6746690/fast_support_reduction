#include <iostream>
#include <stan/math.hpp>
#include <Eigen/Core>
#include <boost/math/tools/promotion.hpp>

using namespace Eigen;
using namespace std;

// template <typename T1, typename T2, typename T3>
// inline
// typename boost::math::tools::promote_args<T1, T2, T3>::type
// normal_log(const T1& y, const T2& mu, const T3& sigma) {
//     using std::pow; using std::log;
//     return -0.5 * pow((y - mu) / sigma, 2.0)
//            -log(sigma)
//            -0.5 * log(2 * stan::math::pi());
// }


// int increment(int x) {
//     return x+1;
// }


// class Func {
//     public:
//         void operator() (const string& str) const {
//             cout << str << endl;
//         }
// };


// class StringAppend {
//     public:
//         explicit StringAppend(const string& str) : ss(str) {}
//         void operator() (const string& str) const {
//             cout << str << " " << ss << endl;
//         }

//     private:
//         const string ss;
// };

// class ShorterThan {
//     public:
//         explicit ShorterThan(int maxLength) : length(maxLength) {}

//         bool operator() (const string& str) const {
//             return str.length() < length;
//         }
//     private:
//         const int length;

// };


// struct normal_ll {
//     const Matrix<double, Dynamic, 1>  y_;

//     normal_ll(const Matrix<double, Dynamic, 1>& y) : y_(y) {}

//     template <typename T>
//     T operator() (const Matrix<T, Dynamic, 1>& theta) const {
//         T mu = theta[0];
//         T sigma = theta[1];
//         T lp = 0;
//         for (int n = 0; n < y_.size(); ++n) {
//             lp += normal_log(y_[n], mu, sigma);
//         }
//         return lp;
//     }
// };

// bool is_greater_than_5(int value) {
//     return (value > 5);
// }


template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> f( const Eigen::Matrix<T, Eigen::Dynamic, 1>& x)
{
  Eigen::SparseMatrix<T> G(2200,2200);
  std::vector<Eigen::Triplet<T> > ijv;
  ijv.emplace_back(0,0,x(0));
  ijv.emplace_back(1,1,x(1));
  ijv.emplace_back(2,2,x(2));
  ijv.emplace_back(3,3,x(3));
  for(int d = 4;d<2200;d++) ijv.emplace_back(d,d,1);
  G.setFromTriplets(ijv.begin(),ijv.end());
  G.makeCompressed();
  return (Eigen::RowVectorXd::Constant(1,2200,1) * G).transpose();
}



int main(int argc, char *argv[])
{

    // Matrix<double, Dynamic, 1> M(2);
    // M << 1.,
    //      2.;

    // std::cout << M.size() << std::endl;


    // std::cout << "log normal(1 | 2, 3)="
    //         << stan::math::normal_log(1, 2, 3)
    //         << std::endl;


    // using std::pow;
    // double y = 1.3;
    // stan::math::var mu = 0.5, sigma = 1.2;

    // stan::math::var lp = 0;
    // lp -= 0.5 * log(2 * stan::math::pi());
    // lp -= log(sigma);
    // lp -= 0.5 * pow((y - mu) / sigma, 2);
    // std::cout << "f(mu, sigma) = " << lp.val() << std::endl;

    // // // way 1
    // // lp.grad();
    // // std::cout << " d.f / d.u = " << mu.adj()
    // //           << " d.f / d.sigma = " << sigma.adj() << std::endl;

    // // way 2
    // std::vector<stan::math::var> theta;
    // theta.push_back(mu);
    // theta.push_back(sigma);
    // std::vector<double> g;
    // lp.grad(theta, g);
    // std::cout << " d.f / d.mu = " << g[0]
    //           << " d.f / d.sigma = " << g[1] << std::endl;


    // double y = 1.3;
    // stan::math::var mu = 0.5, sigma = 1.2;

    // stan::math::var lp = normal_log(y, mu, sigma);

    // int x = 5;
    // int y = increment(x);
    // std::cout << y << std::endl;

    // Func myFunc;
    // myFunc("Hello World!");


    // A(0, 0) = x;

    // for (int i = 0; i < 100; i++) {
    //     y = sin(x+y);
    // }

    // std::cout << "value = " << y.val() << std::endl;

    // y.grad();
    // std::cout << " d.f / d.x = " << x.adj()
    //           << " d.f / d.y = " << y.adj() << std::endl;

    // stan::math::var x = 0.5, y = 0.5;

    // Matrix<stan::math::var, 3, 1> A;
    // A(0, 0) = x;
    // A(0, 1) = 5.0;

    // Matrix<double, Dynamic, Dynamic> A(4, 4);
    // A.resize(5, 5);
    // Matrix<float,3*4,3> M_4(3*4, 3);
    // Matrix<double, Dynamic, 1> B(3);

    // stan::math::var x = 0.5, y = 0.5;
    // Matrix<stan::math::var, 3, 1> A;
    // A(0, 0) = x;
    // A(1, 0) = 5.0;
    // A(2, 0) = x*x+y;

    // std::cout << A << std::endl;

    // A(2).grad();
    // std::cout << x.adj() << std::endl;

    // vector<Triplet<stan::math::var>> triplets;

    // auto lambda = [] () {cout << "reach here" << endl;};
    // lambda();

    // auto sum = [] (int x, int y) {return x+y;};
    // cout << sum(2, 3) << endl;
    // cout << sum(3, 4) << endl;

    // vector<int> numbers{1, 5, 10, 20};
    // auto num = count_if(numbers.begin(), numbers.end(), [] (int x) {return x>5;});

    // cout << num << endl;

    // stan::math::var x = 0.5, y = 0.5;
    // Matrix<stan::math::var, 3, 1> A;
    // Matrix<stan::math::var, 1, 3> B;
    // A << 1, 2, 3;
    // B << 4, 5, 6;
    // auto C = B*A;
    
    // cout << C << endl;


    // Matrix<stan::math::var, 3, 1> A;
    // Matrix<double, 1, 3> B;
    // A << 1, 2, 3;
    // B << 4, 5, 6;
    // auto C = B*A;

    // cout << C << endl;

    // stan::math::var a = 5;
    // double b = 3;
    // cout << a*b << endl;


    // Matrix<stan::math::var, 3, 2> A;
    // A << 1, 2, 3,
    //      4, 5, 6;

    // cout << A.row(1) << endl;

    //  SparseMatrix<stan::math::var> K;
    //  K.resize(100, 100);
    //  Matrix<double, Dynamic, 1> u(100);
    //  u.setZero();

    //  auto f = K*u;

    //  cout << f.bottomRows(10) << endl;

    Eigen::VectorXd x = Eigen::VectorXd::Constant(2200,1,1);
    Eigen::MatrixXd J;
    Eigen::VectorXd fx;
    stan::math::jacobian( f<stan::math::var>, x, fx, J);


    return 0;

}