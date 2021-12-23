//
// Created by shuoy on 8/23/21.
//

#include "imu_leg_factor.h"

void IMULegFactor::checkJacobian(const double *const *parameters) {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);


    Eigen::Vector3d Bvi(parameters[2][0], parameters[2][1], parameters[2][2]);

    Eigen::Vector3d Pj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond Qj(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

    Eigen::Vector3d Vj(parameters[4][0], parameters[4][1], parameters[4][2]);
    Eigen::Vector3d Baj(parameters[4][3], parameters[4][4], parameters[4][5]);
    Eigen::Vector3d Bgj(parameters[4][6], parameters[4][7], parameters[4][8]);

    Eigen::Vector3d Bvj(parameters[5][0], parameters[5][1], parameters[5][2]);

    std::vector<int> block_sizes{7,9,3,7,9,3};
    Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 1> residual;
    double **raw_jacobians = new double *[block_sizes.size()];
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    jacobians.resize(block_sizes.size());
    for (int xx = 0; xx < static_cast<int>(block_sizes.size()); xx++)
    {
        jacobians[xx].resize(RESIDUAL_STATE_SIZE, block_sizes[xx]);
        raw_jacobians[xx] = jacobians[xx].data();
        //dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }

    Evaluate(parameters, residual.data(), raw_jacobians);

    // perturb Pi
    Vector3d turb(0.0001, -0.003, 0.003);
    Eigen::Matrix<double, 7, 1> turb_vec; turb_vec.setZero(); turb_vec.segment<3>(0) = turb;

    Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 1> turb_residual =
            il_pre_integration->evaluate(Pi+turb, Qi, Vi, Bai, Bgi, Bvi, Pj, Qj, Vj, Baj, Bgj, Bvj);
    Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE> sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    Eigen::VectorXd tmp = turb_residual - (jacobians[0]* turb_vec + residual);
    std::cout << "residual\t" << residual.transpose() << std::endl;
    std::cout << "turb_residual\t" << turb_residual.transpose() << std::endl;
    std::cout << "jac_residual\t" << (jacobians[0]* turb_vec + residual).transpose() << std::endl;
    std::cout << "perturb Pi\t" << tmp.maxCoeff() << std::endl;

    // perturb Qi
    turb_residual =
            il_pre_integration->evaluate(Pi, Qi* Quaterniond(1, turb(0) / 2, turb(1) / 2, turb(2) / 2).normalized(), Vi, Bai, Bgi, Bvi, Pj, Qj, Vj, Baj, Bgj, Bvj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    turb_vec.setZero(); turb_vec.segment<3>(3) = turb;

    tmp = turb_residual - (jacobians[0]* turb_vec + residual);
    std::cout << "residual\t" << residual.transpose() << std::endl;
    std::cout << "turb_residual\t" << turb_residual.transpose() << std::endl;
    std::cout << "jac_residual\t" << (jacobians[0]* turb_vec + residual).transpose() << std::endl;
    std::cout << "perturb Qi\t" << tmp.transpose() << std::endl;

    // perturb Vi
    Eigen::Matrix<double, 9, 1> turb_vec9; turb_vec9.setZero(); turb_vec9.segment<3>(0) = turb;

    turb_residual =
            il_pre_integration->evaluate(Pi, Qi, Vi+turb, Bai, Bgi, Bvi, Pj, Qj, Vj, Baj, Bgj, Bvj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

//    std::cout << "jacobians[1]\n" << jacobians[1] << std::endl;
    tmp = turb_residual - (jacobians[1]* turb_vec9 + residual);
    std::cout << "perturb Vi\t" << tmp.maxCoeff() << std::endl;

    // perturb Bai
    turb_vec9.setZero(); turb_vec9.segment<3>(3) = turb;

    turb_residual =
            il_pre_integration->evaluate(Pi, Qi, Vi, Bai+turb, Bgi, Bvi, Pj, Qj, Vj, Baj, Bgj, Bvj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

//    std::cout << "jacobians[1]\n" << jacobians[1] << std::endl;
    tmp = turb_residual - (jacobians[1]* turb_vec9 + residual);
    std::cout << "perturb Bai\t" << tmp.maxCoeff() << std::endl;

    // perturb Bgi
    turb_vec9.setZero(); turb_vec9.segment<3>(6) = turb;

    turb_residual =
            il_pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi+turb, Bvi, Pj, Qj, Vj, Baj, Bgj, Bvj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

//    std::cout << "jacobians[1]\n" << jacobians[1] << std::endl;
    tmp = turb_residual - (jacobians[1]* turb_vec9 + residual);
    std::cout << "perturb Bgi\t" << tmp.maxCoeff() << std::endl;

    // perturb rhoi
    Vector3d bvi_rand = 0.003*Eigen::Matrix<double,3,1>::Random();
    turb_residual =
            il_pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Bvi+bvi_rand, Pj, Qj, Vj, Baj, Bgj, Bvj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[2]* bvi_rand + residual);
    std::cout << "perturb rhoi\t" << tmp.maxCoeff() << std::endl;

    // perturb Pj
    turb_vec.setZero(); turb_vec.segment<3>(0) = turb;

    turb_residual =
            il_pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Bvi, Pj+turb, Qj, Vj, Baj, Bgj, Bvj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[3]* turb_vec + residual);
    std::cout << "perturb Pj\t" << tmp.transpose() << std::endl;

    // perturb Qj
    turb_residual =
            il_pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Bvi, Pj, Qj* Quaterniond(1, turb(0) / 2, turb(1) / 2, turb(2) / 2).normalized(), Vj, Baj, Bgj, Bvj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    turb_vec.setZero(); turb_vec.segment<3>(3) = turb;
    tmp = (jacobians[3]* turb_vec + residual);

    tmp = turb_residual - tmp;
    std::cout << "perturb Qj\t" << tmp.transpose() << std::endl;



    // perturb rhoi
    bvi_rand = 0.003*Eigen::Matrix<double,3,1>::Random();
    turb_residual =
            il_pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Bvi, Pj, Qj, Vj, Baj, Bgj, Bvj+bvi_rand);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[5]* bvi_rand + residual);
    std::cout << "perturb Bvj\t" << tmp.maxCoeff() << std::endl;

    return;
}

bool IMULegFactor::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Bvi(parameters[2][0], parameters[2][1], parameters[2][2]);

    Eigen::Vector3d Pj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond Qj(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

    Eigen::Vector3d Vj(parameters[4][0], parameters[4][1], parameters[4][2]);
    Eigen::Vector3d Baj(parameters[4][3], parameters[4][4], parameters[4][5]);
    Eigen::Vector3d Bgj(parameters[4][6], parameters[4][7], parameters[4][8]);

    Eigen::Vector3d Bvj(parameters[5][0], parameters[5][1], parameters[5][2]);

    Eigen::Map<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 1>> residual(residuals);
    residual = il_pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Bvi,
                                            Pj, Qj, Vj, Baj, Bgj, Bvj);
    Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE> sqrt_info = Eigen::LLT<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, RESIDUAL_STATE_SIZE>>(
            il_pre_integration->covariance.inverse()).matrixL().transpose();
//        sqrt_info.setIdentity();
    residual = sqrt_info * residual;

    if (jacobians) {
        double sum_dt = il_pre_integration->sum_dt;
        Eigen::Matrix3d dp_dba = il_pre_integration->jacobian.template block<3, 3>(ILO_P, ILO_BA);
        Eigen::Matrix3d dp_dbg = il_pre_integration->jacobian.template block<3, 3>(ILO_P, ILO_BG);

        Eigen::Matrix3d dq_dbg = il_pre_integration->jacobian.template block<3, 3>(ILO_R, ILO_BG);

        Eigen::Matrix3d dv_dba = il_pre_integration->jacobian.template block<3, 3>(ILO_V, ILO_BA);
        Eigen::Matrix3d dv_dbg = il_pre_integration->jacobian.template block<3, 3>(ILO_V, ILO_BG);

        Eigen::Matrix3d dep_dbg = il_pre_integration->jacobian.template block<3, 3>(ILO_EPS, ILO_BG);
        Eigen::Matrix3d dep_dbv = il_pre_integration->jacobian.template block<3, 3>(ILO_EPS, ILO_BV);

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();

            jacobian_pose_i.block<3, 3>(ILO_P, 0) = -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(ILO_P, 3) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

            Eigen::Quaterniond corrected_delta_q = il_pre_integration->delta_q * Utility::deltaQ(
                    dq_dbg * (Bgi - il_pre_integration->linearized_bg));
            jacobian_pose_i.block<3, 3>(ILO_R, 3) = -(Utility::Qleft(Qj.inverse() * Qi) *
                                                      Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
            jacobian_pose_i.block<3, 3>(ILO_V, 3) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

            // related to epsilon
            jacobian_pose_i.block<3, 3>(ILO_EPS, 0) = -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(ILO_EPS, 3) = Utility::skewSymmetric(Qi.inverse() * (Pj - Pi));


            jacobian_pose_i = sqrt_info * jacobian_pose_i;

            if (jacobian_pose_i.maxCoeff() > 1e10 || jacobian_pose_i.minCoeff() < -1e10) {
                ROS_WARN("numerical unstable in preintegration");
                std::cout << sqrt_info << std::endl;
                ROS_BREAK();
            }
        }

        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
            jacobian_speedbias_i.setZero();
            jacobian_speedbias_i.block<3, 3>(ILO_P, 0) = -Qi.inverse().toRotationMatrix() * sum_dt;
            jacobian_speedbias_i.block<3, 3>(ILO_P, 3) = -dp_dba;
            jacobian_speedbias_i.block<3, 3>(ILO_P, 6) = -dp_dbg;
            //
            jacobian_speedbias_i.block<3, 3>(ILO_R, 6) =
                    -Utility::Qleft(Qj.inverse() * Qi * il_pre_integration->delta_q).bottomRightCorner<3, 3>() *
                    dq_dbg;

            jacobian_speedbias_i.block<3, 3>(ILO_V, 0) = -Qi.inverse().toRotationMatrix();
            jacobian_speedbias_i.block<3, 3>(ILO_V, 3) = -dv_dba;
            jacobian_speedbias_i.block<3, 3>(ILO_V, 6) = -dv_dbg;

//            jacobian_speedbias_i.block<3, 3>(ILO_EPS1, 0) = -Qi.inverse().toRotationMatrix();
            jacobian_speedbias_i.block<3, 3>(ILO_EPS, 6) = -dep_dbg;

            jacobian_speedbias_i.block<3, 3>(ILO_BA, 3) = -Eigen::Matrix3d::Identity();
            jacobian_speedbias_i.block<3, 3>(ILO_BG, 6) = -Eigen::Matrix3d::Identity();

//            std::cout << "jacobian_speedbias_i" << std::endl;
//            std::cout << jacobian_speedbias_i << std::endl;
            jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
//            std::cout << fabs(jacobian_speedbias_i.maxCoeff()) << std::endl;
            ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e10);
            ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e10);
            if (jacobian_speedbias_i.maxCoeff() > 1e10 || jacobian_speedbias_i.minCoeff() < -1e10) {
                ROS_WARN("numerical unstable in preintegration");
                std::cout << sqrt_info << std::endl;
                ROS_BREAK();
            }
        }

        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 3, Eigen::RowMajor>> jacobian_legbias_i(jacobians[2]);
            jacobian_legbias_i.setZero();

            jacobian_legbias_i.block<3, 3>(ILO_EPS, 0) = -dep_dbv;
            jacobian_legbias_i.block<3, 3>(ILO_BV, 0) = -Eigen::Matrix3d::Identity();

            jacobian_legbias_i = sqrt_info * jacobian_legbias_i;
//            ROS_ASSERT(fabs(jacobian_legbias_i.maxCoeff()) < 1e8);
//            ROS_ASSERT(fabs(jacobian_legbias_i.minCoeff()) < -1e10);
            if (jacobian_legbias_i.maxCoeff() > 1e10 || jacobian_legbias_i.minCoeff() < -1e10) {
                ROS_WARN("numerical unstable in preintegration");
                std::cout << sqrt_info << std::endl;
                ROS_BREAK();
            }
        }

        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[3]);
            jacobian_pose_j.setZero();

            jacobian_pose_j.block<3, 3>(ILO_P, 0) = Qi.inverse().toRotationMatrix();
            Eigen::Quaterniond corrected_delta_q = il_pre_integration->delta_q * Utility::deltaQ(
                    dq_dbg * (Bgi - il_pre_integration->linearized_bg));
            jacobian_pose_j.block<3, 3>(ILO_R, 3) = Utility::Qleft(
                    corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();


            jacobian_pose_j.block<3, 3>(ILO_EPS, 0) = Qi.inverse().toRotationMatrix();

            jacobian_pose_j = sqrt_info * jacobian_pose_j;
//            ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
//            ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            if (jacobian_pose_j.maxCoeff() > 1e10 || jacobian_pose_j.minCoeff() < -1e10) {
                ROS_WARN("numerical unstable in preintegration");
                std::cout << sqrt_info << std::endl;
                ROS_BREAK();
            }
        }

        if (jacobians[4]) {
            Eigen::Map<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[4]);
            jacobian_speedbias_j.setZero();

            jacobian_speedbias_j.block<3, 3>(ILO_V, 0) = Qi.inverse().toRotationMatrix();

            jacobian_speedbias_j.block<3, 3>(ILO_BA, 3) = Eigen::Matrix3d::Identity();

            jacobian_speedbias_j.block<3, 3>(ILO_BG, 6) = Eigen::Matrix3d::Identity();

            jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

//            ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
//            ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            if (jacobian_speedbias_j.maxCoeff() > 1e10 || jacobian_speedbias_j.minCoeff() < -1e10) {
                ROS_WARN("numerical unstable in preintegration");
                std::cout << sqrt_info << std::endl;
                ROS_BREAK();
            }
        }
        if (jacobians[5]) {
            Eigen::Map<Eigen::Matrix<double, RESIDUAL_STATE_SIZE, 3, Eigen::RowMajor>> jacobian_legbias_j(jacobians[5]);
            jacobian_legbias_j.setZero();

            jacobian_legbias_j.block<3, 3>(ILO_BV, 0) = Eigen::Matrix3d::Identity();

            jacobian_legbias_j = sqrt_info * jacobian_legbias_j;
//            ROS_ASSERT(fabs(jacobian_legbias_j.maxCoeff()) < 1e8);
//            ROS_ASSERT(fabs(jacobian_legbias_j.minCoeff()) < 1e8);
            if (jacobian_legbias_j.maxCoeff() > 1e10 || jacobian_legbias_j.minCoeff() < -1e10) {
                ROS_WARN("numerical unstable in preintegration");
                std::cout << sqrt_info << std::endl;
                ROS_BREAK();
            }
        }
    }
    return true;
}
