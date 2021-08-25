//
// Created by shuoy on 7/19/21.
//

#include <ceres/ceres.h>

#include "../legKinematics/A1Kinematics.h"

int main(int argc, char **argv) {

    A1Kinematics a1_kin;
    Eigen::Vector3d q(0.1,0.1,0.3);
    Eigen::VectorXd rho_fix(5); rho_fix << 0.3,0.15,0.08,0.2,0.2;
    Eigen::VectorXd rho_opt(3); rho_opt << 0.0,0.0,0.0;
    Eigen::VectorXd rho_perturb(3); rho_perturb << 0.003,0.0,-0.001;
    Eigen::VectorXd _dphi_0(3); _dphi_0 << 1.1,2.2,-3.3;
    Eigen::Vector3d p_bf = a1_kin.fk(q, rho_opt,rho_fix);
    std::cout << "forward kinematics \n" << p_bf << std::endl;
    Eigen::MatrixXd Jac = a1_kin.jac(q, rho_opt,rho_fix);
    std::cout << "Jacobian \n" <<Jac << std::endl;

    Eigen::MatrixXd dfk_drho = a1_kin.dfk_drho(q, rho_opt,rho_fix);
    std::cout << "dfk_drho \n" <<dfk_drho << std::endl;

    Eigen::MatrixXd dJ_dq = a1_kin.dJ_dq(q, rho_opt,rho_fix);
    std::cout << "dJ_dq \n" <<dJ_dq << std::endl;
    Eigen::MatrixXd dJ_drho = a1_kin.dJ_drho(q, rho_opt,rho_fix);
    std::cout << "dJ_drho \n" <<dJ_drho << std::endl;

    // kron product test
//    Eigen::MatrixXd dJdrho0 = a1_kin.dJ_drho(q, rho_opt,rho_fix);
//    Eigen::MatrixXd kron_dphi0(3,9); kron_dphi0.setZero();
//    kron_dphi0(0,0) = kron_dphi0(1,1) = kron_dphi0(2,2) = _dphi_0(0);
//    kron_dphi0(0,3) = kron_dphi0(1,4) = kron_dphi0(2,5) = _dphi_0(1);
//    kron_dphi0(0,6) = kron_dphi0(1,7) = kron_dphi0(2,8) = _dphi_0(2);
//
//    Eigen::MatrixXd tmp = dJdrho0*rho_perturb;
//    Eigen::Map<Eigen::MatrixXd> M2(tmp.data(), 3,3);
//    Eigen::VectorXd out1 = M2*_dphi_0;
//
//    Eigen::VectorXd out2 = kron_dphi0*dJdrho0*rho_perturb;
//
//    std::cout << "out1 \n" <<out1 << std::endl;
//    std::cout << "out2 \n" <<out2 << std::endl;

    // test jacobian
    Eigen::MatrixXd Jac1 = a1_kin.jac(q, rho_opt,rho_fix);
    Eigen::MatrixXd Jac2 = a1_kin.jac(q, rho_opt+rho_perturb,rho_fix);
    Eigen::MatrixXd tmp = dJ_drho*rho_perturb;
    Eigen::Map<Eigen::MatrixXd> M2(tmp.data(), 3,3);
    Eigen::MatrixXd residual = Jac2 - (Jac1 + M2);

    std::cout << "residual \n" <<residual << std::endl;
    return 0;
}