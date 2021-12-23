# ex02-3D Rigid Body Motion

## Eigen Matrix Computation

Given linear equation Ax = b, A is square matrix:

1. under which condition, x has unique solution?

   A is full rank matrix, x has a solution and is unique

2. Gaussian Elimation

   Gaussian elimination is an algorithm in linear algebra that can be used to solve systems of linear equations, as well as to find the rank of a matrix, and to find the inverse of an invertible matrix. The Gaussian elimination method is based on the following principle: by transforming the increasing matrix into a row-order array using the elementary row transform, and then solving the solution of the system of linear equations by backbanding.

3. QR Decompostion

   QR decomposition is one of the three ways to decompose a matrix. In this way, the matrix is decomposed into the product of an orthogonal matrix and an upper triangular matrix.QR decomposition is often used to solve linear least squares problems.

4. Cholesky Decomposition

   Cholesky decomposition is the representation of a real symmetric positive definite matrix as a product of a lower triangular matrix L and its transpose. The process of solving the equation becomes.

5.  A is 100 × 100 random matrix, use QR and Cholesky to decompose x

## Geometry Computation

```c++
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;

int main( int argc, char** argv )
{
    Eigen::Quaterniond q_1(0.35,0.2,0.3,0.1);
    Eigen::Quaterniond q_2(-0.5,0.4,-0.1,0.2);
    q_1.normalize();
    q_2.normalize();
    Eigen::Vector3d p_1(0.5,0,0.2);
    Eigen::Vector3d p_2;
    Eigen::Vector3d t_1(0.3,0.1,0.1);
    Eigen::Vector3d t_2(-0.1,0.5,0.3);
    p_2 = q_2*(t_1+q_1.conjugate()*p_1 -t_2 );
cout<<p_2<<endl;
return 0;
}
```


## Rotation Representation

$$
R = \left[ \begin{matrix} e_{1}^{T} \\ e_{2}^{T} \\ e_{3}^{T} \end{matrix} \right]\left[ \begin{matrix} e_{1}^{'} & e_{2}^{'} & e_{3}^{'} \end{matrix} \right]
$$

$$
R^{T} = \left[ \begin{matrix} e_{1}^{'T} \\ e_{2}^{'T} \\ e_{3}^{'T} \end{matrix} \right]\left[ \begin{matrix} e_{1} & e_{2} & e_{3} \end{matrix} \right]
$$

$$
R^{T}R = \left[ \begin{matrix} e_{1}^{'T} \\ e_{2}^{'T} \\ e_{3}^{'T} \end{matrix} \right]\left[ \begin{matrix} e_{1} & e_{2} & e_{3} \end{matrix} \right]\left[ \begin{matrix} e_{1}^{T} \\ e_{2}^{T} \\ e_{3}^{T} \end{matrix} \right]\left[ \begin{matrix} e_{1}^{'} & e_{2}^{'} & e_{3}^{'} \end{matrix} \right]=I
$$

quaternion
$$
q=(\epsilon, \eta)
$$
imagary part 3 dimension
$$
\epsilon 
$$
 1 dimension
$$
\eta
$$

### quaternion multiplication

$$
\begin{aligned}
   q\otimes p =& \begin{bmatrix}
       p_wq_w-p_xq_x-p_yq_y-p_zq_z \\
       p_wq_x+p_xq_w+p_yq_z-p_zq_y \\
       p_wq_y-p_xq_z+p_yq_w+p_zq_x \\
       p_wq_z+p_xq_y-p_yq_x+p_zq_w
   \end{bmatrix} \\
   =& \begin{bmatrix}
       p_wq_w - \pmb{p}_v^T\pmb{q}_v \\
       p_w\pmb{q}_v+q_w\pmb{p}_v+\pmb{p}_v\times \pmb{q}_v

   \end{bmatrix}
\end{aligned}
$$

$$
\pmb{q}_1\otimes \pmb{q}_2=[\pmb{q}_1]_L\pmb{q}_2 \\
\pmb{q}_1\otimes \pmb{q}_2=[\pmb{q}_2]_R\pmb{q}_1
$$

$$
\begin{aligned}
   [\pmb{q}_1]_L=&\begin{bmatrix}
       q_w& -q_z& qy& q_x\\
       q_z& q_w& -q_x& q_y\\
       -q_y& q_x& q_w& q_z\\
       -q_x& -q_y& -q_z& q_w
   \end{bmatrix}\\
   =&\begin{bmatrix}
       \eta1 + \epsilon^{\wedge}& \epsilon \\
       -\epsilon& \eta
   \end{bmatrix}
\end{aligned}
$$

$$
\pmb[q_2]_R
$$

## Rodrigues' rotation formula

$$
RR^T=RR^{-1}=I
$$

$$
det(R)=det(r_{1}, r_{2}, r_{3})=r_{1}\cdot(r_{2}\times r_{3}) = 1
$$



## Quaternion Computation

assume that
$$
q = \left[\begin{matrix}\epsilon \\ \eta  \end{matrix}\right],q^{-1} = \left[\begin{matrix}-\epsilon \\ \eta  \end{matrix}\right]
$$

$$
p = \left[\begin{matrix}r_{b} \\ 0  \end{matrix}\right],p^{-1} = \left[\begin{matrix}r_{b}^{'} \\ 0  \end{matrix}\right]
$$

then
$$
p^{'}=q^{+}q^{-1(+)}p= \left[\begin{matrix}\eta I +\epsilon^{\times} & \epsilon \\ -\epsilon^{T} & \eta \end{matrix}\right]\left[\begin{matrix}\eta I -\epsilon^{\times} & \epsilon \\ -\epsilon^{T} & \eta \end{matrix}\right] \left[\begin{matrix}r_{b} \\ 0  \end{matrix}\right]
= \left[\begin{matrix}-(\eta I -\epsilon^{\times})^{2} & 0 \\ 0 & -1 \end{matrix}\right] \left[\begin{matrix}r_{b} \\ 0  \end{matrix}\right] 
$$
we get
$$
r_{b}^{'}=(-(\eta I-\epsilon^{\times})^{2}-\epsilon\epsilon^{T})r_{b}
$$

$$
R=-(\eta I-\epsilon^{\times})^{2}-\epsilon\epsilon^{T}
$$



## C++11

```c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class A {
public:
A(const int& i ) : index(i) {}
int index = 0;
};

int main() {
	A a1(3), a2(5), a3(9);
    vector<A> avec{a1, a2, a3};
    std::sort(avec.begin(), avec.end(), [](const A&a1, const A&a2) 
              {return a1.index<a2.index;});
    for ( auto& a: avec ) cout<<a.index<<" ";
    cout<<endl;
    return 0;
}
```

