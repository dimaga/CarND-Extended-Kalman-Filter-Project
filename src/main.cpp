#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::string;
using std::ifstream;
using std::ofstream;
using std::endl;
using std::cout;
using std::cerr;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (1 == argc) {
    cerr << usage_instructions << endl;
  } else if (2 == argc) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (3 == argc) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(const ifstream& in_file,
                 const string& in_name,
                 const ofstream& out_file,
                 const string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {
  check_arguments(argc, argv);

  const string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  const string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    std::istringstream iss(line);
    std::int64_t timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      double x;
      double y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      double ro;
      double phi;
      double ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    double x_gt;
    double y_gt;
    double vx_gt;
    double vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    const auto& mpl = measurement_pack_list[k];

    // start filtering from the second frame
    // (the speed is unknown in the first frame)
    fusionEKF.ProcessMeasurement(mpl);

    // output the estimation
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";

    // output the measurements
    if (MeasurementPackage::LASER == mpl.sensor_type_) {
      // output the estimation
      out_file_ << mpl.raw_measurements_(0) << "\t";
      out_file_ << mpl.raw_measurements_(1) << "\t";
    } else if (MeasurementPackage::RADAR == mpl.sensor_type_) {
      // output the estimation in the cartesian coordinates
      double ro = mpl.raw_measurements_(0);
      double phi = mpl.raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t";  // p1_meas
      out_file_ << ro * sin(phi) << "\t";  // ps_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    estimations.push_back(fusionEKF.ekf_.x_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl;
  cout << CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
